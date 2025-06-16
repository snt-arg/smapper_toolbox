"""Kalibr calibration module.

This module provides classes for camera, IMU, and camera-IMU calibration using the Kalibr toolbox.
It handles the execution of calibration procedures in Docker containers and manages the calibration
process for different sensor configurations.
"""

import os
import shutil
from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional

import yaml
from rich.progress import Progress, SpinnerColumn, TextColumn

from smapper_toolbox.config import Config
from smapper_toolbox.logger import logger
from smapper_toolbox.rosbags.analyzer import (
    CalibrationMode,
    RosbagAnalyzer,
    RosbagInfo,
    TopicSelector,
)
from smapper_toolbox.utils import DockerError, DockerRunner
from smapper_toolbox.utils.executor import execute_docker_pool


class CalibrationBase(ABC):
    """Base class for all calibration procedures.

    This abstract class defines the interface for calibration procedures
    and provides common functionality used by all calibration types.

    Args:
        config: Configuration settings for the calibration.
        docker_helper: Helper class for Docker operations.
        data_path: Mount path for data inside Docker container.
        bags_path: Mount path for ROS bags inside Docker container.
        bag_analyzer: Analyzer for ROS bag files.

    Attributes:
        config: Configuration settings for the calibration.
        docker_helper: Helper class for Docker operations.
        docker_data_path: Mount path for data inside Docker container.
        docker_bags_path: Mount path for ROS bags inside Docker container.
        bag_analyzer: Analyzer for ROS bag files.
    """

    def __init__(
        self,
        config: Config,
        docker_helper: DockerRunner,
        data_path: str,
        bags_path: str,
        bag_analyzer: RosbagAnalyzer,
    ):
        self.config = config
        self.docker_helper = docker_helper
        self.docker_data_path = data_path
        self.docker_bags_path = bags_path
        self.bag_analyzer = bag_analyzer

    @abstractmethod
    def run(self) -> None:
        """Execute the calibration procedure.

        This method must be implemented by concrete calibration classes
        to perform their specific calibration procedure.
        """
        pass


class CameraCalibration(CalibrationBase):
    """Handles camera calibration using Kalibr.

    This class implements camera calibration for one or more cameras
    using the Kalibr toolbox. It processes ROS bags with camera data
    and generates calibration files.

    The calibration process:
    1. Finds all calibration bags with camera data
    2. Runs Kalibr calibration for each bag
    3. Moves generated calibration files to the appropriate directory
    """

    def get_calibation_command(
        self, bag_name: str, topics: List[str], rolling_shutter: bool = False
    ) -> Dict[str, Any]:
        """Generate Docker job configuration for camera calibration.

        Args:
            bag_name: Name of the ROS bag file containing camera data.
            topics: List of ROS topics containing camera images.
            rolling_shutter: Whether to use rolling shutter calibration.

        Returns:
            dict: Docker job configuration containing:
                - img_tag: Docker image tag
                - command: Command to run in container
                - env_var: Environment variables
                - volumes: Volume mounts
        """
        exec = (
            "kalibr_calibrate_rs_cameras"
            if rolling_shutter
            else "kalibr_calibrate_cameras"
        )

        target = os.path.join(
            self.docker_data_path,
            self.config.calibrators.camera_calibrator.target_filename,
        )

        camera_model = self.config.calibrators.camera_calibrator.camera_model

        # fmt: off
        cmd = [
            "rosrun", "kalibr", exec,
            "--bag", f"/bags/{bag_name}", "--bag-freq", "10",
            "--target", target,
            "--dont-show-report",
        ]

        # fmt: on

        cmd.append("--topics")
        cmd.extend(topics)

        cmd.append("--models")
        cmd.extend([camera_model for _ in range(len(topics))])

        return {
            "img_tag": "kalibr",
            "command": cmd,
            "env_var": {"DISPLAY": "$DISPLAY"},
            "volumes": [
                "/tmp/.X11-unix:/tmp/.X11-unix:rw",
                f"{self.config.calibration_dir}:{self.docker_data_path}",
                f"{os.path.join(self.config.rosbags_dir, 'ros1')}:{self.docker_bags_path}",
            ],
        }

    def _move_calibration_files(self, save_dir: str) -> None:
        """Move generated calibration files to the save directory.

        Args:
            save_dir: Directory to save the calibration files.
        """
        logger.info(f"Moving generated files into {save_dir}")
        for file in os.listdir(os.path.join(self.config.rosbags_dir, "ros1")):
            file_extension = file.split(".")[-1]

            if file_extension not in ["yaml", "pdf", "txt"]:
                logger.debug(f"Skipping non-calibration file: {file}")
                continue

            file = os.path.join(self.config.rosbags_dir, "ros1", file)
            shutil.move(file, save_dir)

    def run(self) -> None:
        """Run the camera calibration process.

        This method:
        1. Finds all calibration bags with camera data
        2. Runs Kalibr calibration for each bag in parallel
        3. Moves generated calibration files to the save directory
        """
        logger.info("Running camera calibrations.")
        topic_selector = TopicSelector()
        jobs = []

        bags = self.bag_analyzer.find_calibration_bags(
            self.config.rosbags_dir, CalibrationMode.CAMERA_ONLY
        )

        logger.info("The following bags will be used")
        for bag in bags:
            logger.info(f"Processing bag: {bag}")
            jobs.append(
                self.get_calibation_command(
                    bag.name, topic_selector.select_camera_topics(bag)
                )
            )

        execute_docker_pool(
            self.docker_helper,
            jobs,
            "Running calibrations in Kalibr...",
            self.config.calibrators.camera_calibrator.parallel_calibrations,
        )

        save_dir = os.path.join(
            self.config.calibration_dir,
            self.config.calibrators.camera_calibrator.save_dir,
        )

        if not os.path.exists(save_dir):
            os.makedirs(save_dir)

        self._move_calibration_files(save_dir)


class IMUCalibration(CalibrationBase):
    """Handles IMU calibration using Allan Variance analysis.

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
    ) -> Dict[str, Any]:
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
        cmd = [
            "rosrun",
            "kalibr",
            "kalibr_calibrate_imu_camera",
            "--bag",
            f"/bags/{bag_name}",
            "--cams",
            os.path.join(self.docker_data_path, camera_yaml),
            "--imu",
            os.path.join(self.docker_data_path, imu_yaml),
            "--imu-models",
            "calibrated",
            "--target",
            os.path.join(self.docker_data_path, self.config.april_tag_filename),
            "--dont-show-report",
            "--reprojection-sigma",
            "1.0",
        ]

        return {
            "img_tag": "kalibr",
            "command": cmd,
            "env_var": {"DISPLAY": "$DISPLAY"},
            "volumes": [
                "/tmp/.X11-unix:/tmp/.X11-unix:rw",
                f"{self.config.calibration_dir}:{self.docker_data_path}",
                f"{os.path.join(self.config.rosbags_dir, 'ros1')}:{self.docker_bags_path}",
            ],
        }

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


class Calibrators:
    """Main class for managing calibration procedures.

    This class provides a high-level interface for running different types
    of calibrations using the Kalibr toolbox. It manages the setup and
    execution of camera, IMU, and camera-IMU calibrations.

    Args:
        config: Configuration settings for the calibration procedures.

    Attributes:
        config: Configuration settings for the calibration procedures.
        docker_runner: Helper class for Docker operations.
        docker_data_path: Mount path for data inside Docker container.
        docker_bags_path: Mount path for ROS bags inside Docker container.
        bag_analyzer: Analyzer for ROS bag files.
        camera_calibrator: Camera calibration handler.
        imu_calibrator: IMU calibration handler.
        imu_camera_calibrator: Camera-IMU calibration handler.
    """

    def __init__(self, config: Config):
        self.config = config
        self.docker_runner = DockerRunner()
        self.docker_data_path = "/data"
        self.docker_bags_path = "/bags"
        self.bag_analyzer = RosbagAnalyzer()

        self.camera_calibrator = CameraCalibration(
            self.config,
            self.docker_runner,
            self.docker_data_path,
            self.docker_bags_path,
            self.bag_analyzer,
        )
        self.imu_calibrator = IMUCalibration(
            self.config,
            self.docker_runner,
            self.docker_data_path,
            self.docker_bags_path,
            self.bag_analyzer,
        )
        self.imu_camera_calibrator = IMUCameraCalibration(
            self.config,
            self.docker_runner,
            self.docker_data_path,
            self.docker_bags_path,
            self.bag_analyzer,
        )

    def setup(self) -> bool:
        """Set up the calibration environment.

        This method validates the required directories and builds the Kalibr
        Docker image if it doesn't exist.

        Returns:
            bool: True if setup was successful, False otherwise.
        """
        if not self._validate_dirs():
            return False
        if not self._prepare_kalibr_image():
            return False
        return True

    def calibrate_cameras(self) -> None:
        """Run camera calibration."""
        self.camera_calibrator.run()

    def calibrate_imu(self) -> None:
        """Run IMU calibration."""
        self.imu_calibrator.run()

    def calibrate_imu_camera(self) -> None:
        """Run camera-IMU calibration."""
        self.imu_camera_calibrator.run()

    def _prepare_kalibr_image(self) -> bool:
        """Prepare the Kalibr Docker image.

        This method checks if the Kalibr Docker image exists and builds it
        if necessary.

        Returns:
            bool: True if the image is ready, False otherwise.
        """
        if self.docker_runner.image_exists(self.config.kalibr_image_tag):
            logger.info(f"Kalibr Docker image <{self.config.kalibr_image_tag}> found.")
            return True

        logger.info("Kalibr Docker image does not yet exist. Building it")
        with Progress(
            SpinnerColumn(),
            TextColumn("[progress.description]{task.description}"),
            transient=True,
        ) as progress:
            progress.add_task(
                description="Building docker image. (Patience)", total=None
            )
            try:
                self.docker_runner.build_image(
                    tag=self.config.kalibr_image_tag,
                    path=self.config.smapper_dir,
                    dockerfile=os.path.join(
                        self.config.smapper_dir, "docker", "kalibr", "Dockerfile"
                    ),
                )
            except DockerError:
                logger.error("Failed to build Kalibr image")
                return False

            logger.info("Kalibr Docker image has been created!")

        return True

    def _validate_dirs(self) -> bool:
        """Validate required directories and files.

        This method checks if the required directories and files exist
        for the calibration process.

        Returns:
            bool: True if all required files and directories exist, False otherwise.
        """
        calib_dir = self.config.calibration_dir
        smapper_dir = self.config.smapper_dir
        logger.info(f"Validating file contents of {calib_dir}")

        # Check if april tag file exists
        april_tag_path = os.path.join(calib_dir, self.config.april_tag_filename)
        if not os.path.isfile(april_tag_path):
            logger.error(f"April Tag config file {april_tag_path} does not exist")
            return False

        # Check if SMapper repository exists
        if not os.path.isdir(smapper_dir):
            logger.error(f"SMapper directory {smapper_dir} does not exist")
            return False

        return True
