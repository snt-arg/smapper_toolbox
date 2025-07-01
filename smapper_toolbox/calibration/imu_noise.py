import os
import shutil
from typing import List

import yaml
from rich.progress import Progress, SpinnerColumn, TextColumn

from smapper_toolbox.logger import logger
from smapper_toolbox.rosbags.analyzer import (
    CalibrationMode,
    RosbagInfo,
)

from . import IMU_NOISE_FILENAME, CalibrationBase

IMU_CONFIG_FILE = "imu_config.yaml"
IMU_NOISE_RAW = "imu_noise_raw.yaml"
ACCEL_PLOT_PATH = "/catkin_ws/acceleration.png"
GYRO_PLOT_PATH = "/catkin_ws/gyro.png"


class IMUCalibration(CalibrationBase):
    """
    Performs IMU noise calibration using Allan Variance analysis with https://github.com/ori-drs/allan_variance_ros in Docker.

    This class manages the process of calibrating IMU noise parameters from ROS bag data, including container management, Allan Variance computation, result analysis, and result file handling.
    """

    def _create_temp_container(self) -> None:
        """
        Create a temporary persistent Docker container for running ROS commands.
        """
        logger.info("Creating temporary persistent container for roscore")
        self.docker_helper.create_persistent_container(
            "kalibr",
            "kalibr",
            ["roscore"],
            env_var={"DISPLAY": "$DISPLAY"},
            volumes=[
                "/tmp/.X11-unix:/tmp/.X11-unix:rw",
                f"{self.config.workspace.calibration_dir}:{self.docker_calibration_dir}",
                f"{self.rosbags_dir}:{self.docker_rosbags_dir}",
            ],
        )

    def _cleanup_temp_container(self) -> None:
        """
        Clean up the temporary Docker container used for IMU calibration.
        """
        logger.info("Cleaning up temporary container")
        self.docker_helper.cleanup_container(container_name="kalibr")

    def _run_container_command(self, cmd: List[str], description: str) -> bool:
        """
        Run a command in the temporary container and track progress.

        Args:
            cmd (List[str]): Command to run in the container.
            description (str): Description for the progress bar.

        Returns:
            bool: True if the command was successful, False otherwise.
        """
        with Progress(
            SpinnerColumn(),
            TextColumn("[progress.description]{task.description}"),
            transient=True,
        ) as progress:
            progress.add_task(description=description, total=None)
            try:
                container = self.docker_helper.client.containers.get("kalibr")
                result = container.exec_run(cmd)
                if result.exit_code != 0:
                    logger.error(f"Command failed: {description}")
                    logger.error(f"Error output: {result.output.decode()}")
                    self._cleanup_temp_container()
                    return False
            except Exception as e:
                logger.error(f"Error running command {description}: {str(e)}")
                self._cleanup_temp_container()
                return False
        return True

    def _compute_allan_variance(self, bag_name: str) -> bool:
        """
        Compute Allan Variance for the IMU data inside the Docker container.

        Returns:
            bool: True if the computation was successful, False otherwise.
        """

        # Inside Docker
        imu_config_file = os.path.join(
            self.docker_calibration_dir,
            self.config.calibration.imu.save_dir,
            bag_name,
            IMU_CONFIG_FILE,
        )

        cmd = [
            "bash",
            "-c",
            " && ".join(
                [
                    "source /opt/ros/noetic/setup.bash",
                    "source /catkin_ws/devel/setup.bash",
                    f"rosrun allan_variance_ros allan_variance /bags/temp {imu_config_file}",
                ],
            ),
        ]

        return self._run_container_command(cmd, "Running Allan Variance...")

    def _analyze_allan_variance(self, bag_name: str) -> bool:
        """
        Analyze the Allan Variance data and generate calibration parameters inside the Docker container.

        Returns:
            bool: True if the analysis was successful, False otherwise.
        """

        # Inside Docker
        save_dir = os.path.join(
            self.docker_calibration_dir,
            self.config.calibration.imu.save_dir,
            bag_name,
        )
        imu_config_file = os.path.join(
            save_dir,
            IMU_CONFIG_FILE,
        )

        imu_out_file = os.path.join(
            save_dir,
            IMU_NOISE_RAW,
        )

        cmd = [
            "bash",
            "-c",
            " && ".join(
                [
                    "source /opt/ros/noetic/setup.bash",
                    "source /catkin_ws/devel/setup.bash",
                    f"""rosrun allan_variance_ros analysis.py \
                    --data /bags/temp/allan_variance.csv \
                    --output {imu_out_file} \
                    --config {imu_config_file}""",
                ]
            ),
        ]

        return self._run_container_command(cmd, "Analyzing Allan Variance...")

    def _move_generated_plots(self, bag_name: str) -> bool:
        """
        Move generated IMU Allan Variance plots to the save directory inside the Docker container.

        Returns:
            bool: True if the move was successful, False otherwise.
        """

        # Inside Docker
        save_dir = os.path.join(
            self.docker_calibration_dir,
            self.config.calibration.imu.save_dir,
            bag_name,
        )

        cmd = [
            "bash",
            "-c",
            " && ".join(
                [f"mv {ACCEL_PLOT_PATH} {save_dir}", f"mv {GYRO_PLOT_PATH} {save_dir}"]
            ),
        ]

        return self._run_container_command(
            cmd, f"Moving generated plots into {save_dir}"
        )

    def _update_imu_parameters(self, imu_out_file: str, imu_new_file: str) -> None:
        """
        Update IMU parameters with scaled values and save to a new file.

        Args:
            imu_out_file (str): Path to the IMU calibration file to read.
            imu_new_file (str): Path to the new IMU calibration file to write.
        """
        with open(imu_out_file, "r") as file:
            data = yaml.safe_load(file)

        # Scale values
        data["accelerometer_noise_density"] *= 5
        data["gyroscope_noise_density"] *= 5
        data["accelerometer_random_walk"] *= 10
        data["gyroscope_random_walk"] *= 10

        with open(imu_new_file, "w") as file:
            yaml.safe_dump(data, file)

    def run_single_calibration(self, rosbag: RosbagInfo) -> bool:
        """
        Run calibration for a single IMU bag.

        Args:
            rosbag (RosbagInfo): Information about the ROS bag to process.

        Returns:
            bool: True if calibration was successful, False otherwise.
        """

        if len(rosbag.imu_topics) > 1:
            logger.warning(
                "Current rosbag contains more than 1 IMU topic. Choosing first one available"
            )

        topic = rosbag.imu_topics[0]

        # Create IMU configuration
        imu_config = {
            "imu_topic": topic.name,
            "imu_rate": int(topic.frequency),
            "measure_rate": int(topic.frequency),
            "sequence_time": int(rosbag.duration * 1e-9),
        }

        bag_name = rosbag.name.split(".")[0]

        save_dir = os.path.join(
            self.config.workspace.calibration_dir,
            self.config.calibration.imu.save_dir,
            bag_name,
        )

        os.makedirs(os.path.join(save_dir), exist_ok=True)

        imu_config_file = os.path.join(save_dir, "imu_config.yaml")

        with open(imu_config_file, "w+") as file:
            yaml.safe_dump(imu_config, file)

        # Run calibration steps
        if not all(
            [
                self._compute_allan_variance(bag_name),
                self._analyze_allan_variance(bag_name),
                self._move_generated_plots(bag_name),
            ]
        ):
            return False

        # Update IMU parameters
        imu_out_file = os.path.join(
            save_dir,
            IMU_NOISE_RAW,
        )

        imu_new_file = os.path.join(
            save_dir,
            IMU_NOISE_FILENAME,
        )
        self._update_imu_parameters(imu_out_file, imu_new_file)

        return True

    def run(self, **kwargs) -> None:
        """
        Run the IMU calibration process for all suitable bags.

        This method creates necessary directories, processes each IMU calibration bag, and moves generated files to the appropriate locations.
        """
        del kwargs  # suppressing linter warning

        self.save_dir = os.path.join(
            self.config.workspace.calibration_dir, self.config.calibration.imu.save_dir
        )

        os.makedirs(self.save_dir, exist_ok=True)
        os.makedirs(os.path.join(self.rosbags_dir, "temp"), exist_ok=True)

        bags = self.bag_analyzer.find_calibration_bags(
            self.config.workspace.rosbags_dir, CalibrationMode.IMU_ONLY
        )

        self._create_temp_container()

        for bag in bags:
            print(bag)
            shutil.move(
                os.path.join(self.rosbags_dir, bag.name),
                os.path.join(self.rosbags_dir, "temp"),
            )

            if not self.run_single_calibration(bag):
                continue

            bag_name = bag.name.split(".")[0]

            # Move Allan variance data
            shutil.move(
                f"{self.rosbags_dir}/temp/allan_variance.csv",
                os.path.join(
                    self.save_dir,
                    bag_name,
                    "allan_variance.csv",
                ),
            )
            # Move bag back
            shutil.move(
                os.path.join(self.rosbags_dir, "temp", bag.name),
                os.path.join(self.rosbags_dir),
            )

        self._cleanup_temp_container()

        # Cleanup temp directory
        for file in os.listdir(os.path.join(self.rosbags_dir, "temp")):
            shutil.move(
                os.path.join(self.rosbags_dir, "temp", file),
                os.path.join(self.rosbags_dir, file),
            )
        os.removedirs(os.path.join(self.rosbags_dir, "temp"))
