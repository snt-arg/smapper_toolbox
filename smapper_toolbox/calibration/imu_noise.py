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

from . import CalibrationBase


class IMUCalibration(CalibrationBase):
    """Handles IMU noise calibration using Allan Variance analysis.

    This class implements IMU calibration using the Allan Variance
    method. It processes ROS bags with IMU data and generates
    calibration parameters.

    The calibration process:
    1. Creates a temporary container for running ROS commands
    2. Computes Allan Variance for each IMU
    3. Analyzes the Allan Variance data
    4. Generates calibration parameters and plots
    """

    def _cleanup_temp_container(self) -> None:
        """Clean up the temporary Docker container used for IMU calibration."""
        logger.info("Cleaning up temporary container")
        self.docker_helper.cleanup_container(container_name="kalibr")

    def _create_temp_container(self) -> None:
        """Create a temporary container for running ROS commands."""
        logger.info("Creating temporary persistent container for roscore")
        self.docker_helper.create_persistent_container(
            "kalibr",
            "kalibr",
            ["roscore"],
            env_var={"DISPLAY": "$DISPLAY"},
            volumes=[
                "/tmp/.X11-unix:/tmp/.X11-unix:rw",
                f"{self.config.calibration_dir}:{self.docker_data_path}",
                f"{os.path.join(self.config.rosbags_dir, 'ros1')}:{self.docker_bags_path}",
            ],
        )

    def _run_container_command(self, cmd: List[str], description: str) -> bool:
        """Run a command in the temporary container.

        Args:
            cmd: Command to run in the container.
            description: Description for the progress bar.

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

    def _compute_allan_variance(self) -> bool:
        """Compute Allan Variance for the IMU data.

        Returns:
            bool: True if the computation was successful, False otherwise.
        """
        imu_config_file = os.path.join(
            self.docker_data_path,
            self.config.calibrators.imu_calibrator.save_dir,
            self.config.calibrators.imu_calibrator.imu_config_filename,
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

    def _analyze_allan_variance(self) -> bool:
        """Analyze the Allan Variance data and generate calibration parameters.

        Returns:
            bool: True if the analysis was successful, False otherwise.
        """
        imu_config_file = os.path.join(
            self.docker_data_path,
            self.config.calibrators.imu_calibrator.save_dir,
            self.config.calibrators.imu_calibrator.imu_config_filename,
        )

        imu_out_file = os.path.join(
            self.docker_data_path,
            self.config.calibrators.imu_calibrator.save_dir,
            "imu_out.yaml",
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

    def _move_generated_plots(self) -> bool:
        """Move generated plots to the save directory.

        Returns:
            bool: True if the move was successful, False otherwise.
        """
        accel_plot = "/catkin_ws/acceleration.png"
        gyro_plot = "/catkin_ws/gyro.png"

        save_dir = os.path.join(
            self.docker_data_path, self.config.calibrators.imu_calibrator.save_dir
        )

        cmd = [
            "bash",
            "-c",
            " && ".join([f"mv {accel_plot} {save_dir}", f"mv {gyro_plot} {save_dir}"]),
        ]

        return self._run_container_command(cmd, "Moving generated plots into save_dir")

    def _update_imu_parameters(self, imu_out_file: str, imu_new_file: str) -> None:
        """Update IMU parameters with scaled values.

        Args:
            imu_out_file: Path to the IMU calibration file.
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
        """Run calibration for a single IMU.

        Args:
            rosbag: Information about the ROS bag to process.

        Returns:
            bool: True if calibration was successful, False otherwise.
        """
        self._create_temp_container()

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

        save_dir = os.path.join(
            self.config.calibration_dir, self.config.calibrators.imu_calibrator.save_dir
        )
        imu_config_file = os.path.join(save_dir, "imu_config.yaml")

        with open(imu_config_file, "w") as file:
            yaml.safe_dump(imu_config, file)

        # Run calibration steps
        if not all(
            [
                self._compute_allan_variance(),
                self._analyze_allan_variance(),
                self._move_generated_plots(),
            ]
        ):
            self._cleanup_temp_container()
            return False

        self._cleanup_temp_container()

        # Update IMU parameters
        imu_out_file = os.path.join(
            self.config.calibration_dir,
            self.config.calibrators.imu_calibrator.save_dir,
            "imu_out.yaml",
        )

        imu_new_file = os.path.join(
            self.config.calibration_dir,
            self.config.calibrators.imu_calibrator.save_dir,
            "imu.yaml",
        )
        self._update_imu_parameters(imu_out_file, imu_new_file)

        return True

    def run(self) -> None:
        """Run the IMU calibration process.

        This method:
        1. Creates necessary directories
        2. Processes each IMU calibration bag
        3. Moves generated files to the appropriate locations
        """
        ros1_bags_dir = os.path.join(self.config.rosbags_dir, "ros1")
        os.makedirs(os.path.join(ros1_bags_dir, "temp"), exist_ok=True)
        os.makedirs(
            os.path.join(
                self.config.calibration_dir,
                self.config.calibrators.imu_calibrator.save_dir,
            ),
            exist_ok=True,
        )

        bags = self.bag_analyzer.find_calibration_bags(
            self.config.rosbags_dir, CalibrationMode.IMU_ONLY
        )

        logger.info("The following bags will be used")
        for bag in bags:
            # Move bag to temp directory
            shutil.move(
                os.path.join(ros1_bags_dir, bag.name),
                os.path.join(ros1_bags_dir, "temp"),
            )

            if self.run_single_calibration(bag):
                # Move Allan variance data
                shutil.move(
                    f"{ros1_bags_dir}/temp/allan_variance.csv",
                    os.path.join(
                        self.config.calibration_dir,
                        "static",
                        "imu",
                        "allan_variance.csv",
                    ),
                )

            # Move bag back
            shutil.move(
                os.path.join(ros1_bags_dir, "temp", bag.name),
                os.path.join(ros1_bags_dir),
            )

        # Cleanup temp directory
        for file in os.listdir(os.path.join(ros1_bags_dir, "temp")):
            shutil.move(
                os.path.join(ros1_bags_dir, "temp", file),
                os.path.join(ros1_bags_dir, file),
            )
        os.removedirs(os.path.join(ros1_bags_dir, "temp"))
