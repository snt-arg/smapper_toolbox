import os
import shutil
from typing import Optional

from smapper_toolbox.logger import logger
from smapper_toolbox.rosbags.analyzer import (
    CalibrationMode,
)
from smapper_toolbox.utils.executor import DockerJobConfig, execute_docker_pool

from . import CalibrationBase


class IMUCameraCalibration(CalibrationBase):
    """Handles camera-IMU calibration using Kalibr.

    This class implements the calibration of the transformation
    between cameras and IMU using the Kalibr toolbox. It requires
    both camera and IMU calibration to be completed first.

    The calibration process:
    1. Finds all camera-IMU calibration bags
    2. Verifies existence of required calibration files
    3. Runs Kalibr calibration for each bag
    4. Moves generated calibration files to the appropriate directory
    """

    def run_single_calibration(
        self, bag_name: str, camera_yaml: str, imu_yaml: str
    ) -> DockerJobConfig:
        """Generate Docker job configuration for camera-IMU calibration.

        Args:
            bag_name: Name of the ROS bag file containing synchronized data.
            camera_yaml: Path to the camera calibration YAML file.
            imu_yaml: Path to the IMU calibration YAML file.

        Returns:
            dict: Docker job configuration containing:
                - img_tag: Docker image tag
                - command: Command to run in container
                - env_var: Environment variables
                - volumes: Volume mounts
        """
        # fmt: off
        cmd = [
            "rosrun", "kalibr", "kalibr_calibrate_imu_camera",
            "--bag", f"/bags/{bag_name}",
            "--cams", os.path.join(self.docker_data_path, camera_yaml),
            "--imu", os.path.join(self.docker_data_path, imu_yaml),
            "--imu-models", "calibrated",
            "--target", os.path.join(self.docker_data_path, self.config.april_tag_filename),
            "--reprojection-sigma", "1.0", 
            "--dont-show-report" 
        ]
        # fmt: on

        job_config = DockerJobConfig(
            img_tag="kalibr",
            command=cmd,
            env_var={"DISPLAY": "$DISPLAY"},
            volumes=[
                "/tmp/.X11-unix:/tmp/.X11-unix:rw",
                f"{self.config.calibration_dir}:{self.docker_data_path}",
                f"{os.path.join(self.config.rosbags_dir, 'ros1')}:{self.docker_bags_path}",
            ],
        )

        return job_config

    def _find_camera_yaml(self, bag_name: str, calib_files_path: str) -> Optional[str]:
        """Find the camera calibration YAML file for a given bag.

        Args:
            bag_name: Name of the ROS bag file.
            calib_files_path: Path to the calibration files directory.

        Returns:
            Optional[str]: Path to the camera calibration YAML file, or None if not found.
        """
        for file in os.listdir(calib_files_path):
            if file.endswith(".yaml") and bag_name.split(".")[0] in file:
                return os.path.join(
                    self.config.calibrators.camera_calibrator.save_dir, file
                )
        return None

    def _move_calibration_files(self) -> None:
        """Move generated calibration files to the static directory."""
        logger.info(
            f"Moving generated files into {os.path.join(self.config.calibration_dir, 'static')}"
        )
        for file in os.listdir(os.path.join(self.config.rosbags_dir, "ros1")):
            if not file.endswith((".yaml", ".pdf", ".txt")):
                continue

            file_path = os.path.join(self.config.rosbags_dir, "ros1", file)
            shutil.move(file_path, os.path.join(self.config.calibration_dir, "static"))

    def run(self) -> None:
        """Run the camera-IMU calibration process.

        This method:
        1. Finds all camera-IMU calibration bags
        2. Verifies existence of required calibration files
        3. Runs Kalibr calibration for each bag in parallel
        4. Moves generated calibration files to the static directory
        """
        logger.info(
            "Running camera-IMU calibrations. Only rosbags with prefix cam_imu are considered!"
        )
        jobs = []

        calib_files_path = os.path.join(
            self.config.calibration_dir,
            self.config.calibrators.camera_calibrator.save_dir,
        )
        imu_yaml = os.path.join(
            self.config.calibrators.imu_calibrator.save_dir,
            "imu.yaml",
        )

        bags = self.bag_analyzer.find_calibration_bags(
            self.config.rosbags_dir, CalibrationMode.CAMERA_IMU
        )

        for bag in bags:
            camera_yaml = self._find_camera_yaml(bag.name, calib_files_path)
            if camera_yaml is None:
                logger.error(
                    "No camera calibration files found. Please run camera calibration first."
                )
                return

            if not os.path.exists(os.path.join(self.config.calibration_dir, imu_yaml)):
                logger.error(
                    "No IMU calibration file found. Please run IMU calibration first."
                )
                return

            jobs.append(self.run_single_calibration(bag.name, camera_yaml, imu_yaml))

        execute_docker_pool(
            self.docker_helper,
            jobs,
            "Running camera-IMU calibrations in Kalibr...",
            self.config.calibrators.camera_imu_calibrator.parallel_calibrations,
        )

        self._move_calibration_files()
