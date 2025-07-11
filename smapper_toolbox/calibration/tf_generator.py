from dataclasses import dataclass
from typing import Dict

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R


@dataclass
class Transform:
    parent: str
    child: str
    translation: np.ndarray  # shape (3,)
    rotation: np.ndarray  # quaternion [x, y, z, w]


class TFTreeGenerator:
    def __init__(self):
        self.transforms: Dict[str, Transform] = {}

    def matrix_to_transform(self, matrix, parent_frame, child_frame):
        translation = matrix[:3, 3]
        rotation_matrix = matrix[:3, :3]
        quat = R.from_matrix(rotation_matrix).as_quat()  # [x, y, z, w]
        return Transform(parent_frame, child_frame, translation, quat)

    def invert_transform_matrix(self, matrix):
        R_mat = matrix[:3, :3]
        t = matrix[:3, 3]
        R_inv = R_mat.T
        t_inv = -R_inv @ t

        matrix_inv = np.eye(4)
        matrix_inv[:3, :3] = R_inv
        matrix_inv[:3, 3] = t_inv
        return matrix_inv

    def add_transform_from_matrix(self, matrix, parent_frame, child_frame):
        transform = self.matrix_to_transform(matrix, parent_frame, child_frame)
        self.transforms[f"{parent_frame}_to_{child_frame}"] = transform
        return transform

    def parse_yaml_config(self, config_data):
        transforms = {}
        for key, matrix_list in config_data.items():
            if isinstance(matrix_list, list) and len(matrix_list) == 4:
                matrix = np.array(matrix_list, dtype=float)
                if "-" in key:
                    parent, child = key.split("-", 1)
                    transforms[key] = {
                        "matrix": matrix,
                        "parent": parent,
                        "child": child,
                    }
        return transforms

    def calculate_transforms_from_config(self, config_data):
        parsed = self.parse_yaml_config(config_data)

        direct = [
            "base_link-os_sensor",
            "os_sensor-os_lidar",
            "os_sensor-os_imu",
            "base_link-realsense_link",
        ]
        for key in direct:
            if key in parsed:
                info = parsed[key]
                self.add_transform_from_matrix(
                    info["matrix"], info["parent"], info["child"]
                )

        camera_keys = [
            "front_left-os_imu",
            "front_right-os_imu",
            "side_left-os_imu",
            "side_right-os_imu",
        ]
        base_to_os_sensor = parsed.get("base_link-os_sensor", {}).get(
            "matrix", np.eye(4)
        )
        os_sensor_to_os_imu = parsed.get("os_sensor-os_imu", {}).get(
            "matrix", np.eye(4)
        )
        base_to_os_imu = base_to_os_sensor @ os_sensor_to_os_imu

        for key in camera_keys:
            if key in parsed:
                info = parsed[key]
                cam_to_os_imu = info["matrix"]
                os_imu_to_cam = self.invert_transform_matrix(cam_to_os_imu)
                base_to_cam = base_to_os_imu @ os_imu_to_cam
                self.add_transform_from_matrix(base_to_cam, "base_link", info["parent"])

        if "os_imu-realsense_imu" in parsed:
            imu_to_realsense = parsed["os_imu-realsense_imu"]["matrix"]
            base_to_realsense_imu = base_to_os_imu @ imu_to_realsense

            if "base_link-realsense_link" not in parsed:
                self.add_transform_from_matrix(
                    base_to_realsense_imu, "base_link", "realsense_link"
                )
                self.add_transform_from_matrix(
                    np.eye(4), "realsense_link", "realsense_imu"
                )
            else:
                base_to_realsense_link = parsed["base_link-realsense_link"]["matrix"]
                inv_base_to_realsense = self.invert_transform_matrix(
                    base_to_realsense_link
                )
                realsense_link_to_imu = inv_base_to_realsense @ base_to_realsense_imu
                self.add_transform_from_matrix(
                    realsense_link_to_imu, "realsense_link", "realsense_imu"
                )

    def load_config_from_yaml(self, filename):
        with open(filename, "r") as f:
            return yaml.safe_load(f)

    def print_transform_summary(self):
        print("\n=== TF Tree Transform Summary ===")
        for key, t in self.transforms.items():
            print(f"\n{t.parent} -> {t.child}:")
            print(
                f"  Translation: [{t.translation[0]:.6f}, {t.translation[1]:.6f}, {t.translation[2]:.6f}]"
            )
            print(
                f"  Rotation (quat): [{t.rotation[0]:.6f}, {t.rotation[1]:.6f}, {t.rotation[2]:.6f}, {t.rotation[3]:.6f}]"
            )

    def print_tf_tree_structure(self):
        print("\n=== TF Tree Structure ===")
        print("base_link")
        print("├── os_sensor")
        print("│   ├── os_lidar")
        print("│   └── os_imu")
        print("├── realsense_link")
        print("│   └── realsense_imu")
        print("├── front_left")
        print("├── front_right")
        print("├── side_left")
        print("└── side_right")

    def generate_yaml_config(self, filename="transforms.yaml"):
        config = {"transforms": {}}
        for key, t in self.transforms.items():
            config["transforms"][key] = {
                "parent_frame": t.parent,
                "child_frame": t.child,
                "translation": {
                    "x": float(t.translation[0]),
                    "y": float(t.translation[1]),
                    "z": float(t.translation[2]),
                },
                "rotation": {
                    "x": float(t.rotation[0]),
                    "y": float(t.rotation[1]),
                    "z": float(t.rotation[2]),
                    "w": float(t.rotation[3]),
                },
            }
        with open(filename, "w") as f:
            yaml.dump(config, f, sort_keys=False)
        print(f"YAML config generated: {filename}")

    def generate_launch_file(self, filename="handheld_device_tf.launch.py"):
        """
        Generate a ROS2 launch file with static transform publishers
        """
        launch_content = """from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Static transforms for handheld device
        
"""

        for name, transform in self.transforms.items():
            parent = transform.parent
            child = transform.child
            t = transform.translation
            r = transform.rotation

            launch_content += f"""        # {parent} to {child}
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='{parent}_to_{child}_publisher',
            arguments=['{t[0]:.6f}', '{t[1]:.6f}', '{t[2]:.6f}', 
                      '{r[0]:.6f}', '{r[1]:.6f}', '{r[2]:.6f}', '{r[3]:.6f}',
                      '{parent}', '{child}']
        ),
        
"""

        launch_content += "    ])\n"

        with open(filename, "w") as f:
            f.write(launch_content)

        print(f"\nLaunch file generated: {filename}")
