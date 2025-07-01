"""
Kalibr calibration manager for the SMapper toolbox.

This module provides the Calibrators class, which manages camera, IMU, and camera-IMU calibration workflows using the Kalibr toolbox in Docker containers.
"""

import os

from rich.progress import Progress, SpinnerColumn, TextColumn

from smapper_toolbox.config import Config
from smapper_toolbox.logger import logger
from smapper_toolbox.rosbags.analyzer import (
    RosbagAnalyzer,
)
from smapper_toolbox.utils import DockerError, DockerRunner

from .camera import CameraCalibration
from .camera_imu import IMUCameraCalibration
from .imu_noise import IMUCalibration


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

    def calibrate_cameras(self, rs: bool = False) -> None:
        """Run camera calibration."""
        self.camera_calibrator.run(rs=rs)

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
        if self.docker_runner.image_exists(self.config.docker.image_tag):
            logger.info(f"Kalibr Docker image <{self.config.docker.image_tag}> found.")
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
                dockerfile_path = self.config.docker.dockerfile_path
                if dockerfile_path is None:
                    raise Exception("No dockerfile path provided")
                self.docker_runner.build_image(
                    tag=self.config.docker.image_tag,
                    path=self.config.workspace.base_dir,
                    dockerfile=dockerfile_path,
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
        calib_dir = self.config.workspace.calibration_dir
        smapper_dir = self.config.workspace.base_dir
        logger.info(f"Validating file contents of {calib_dir}")

        # Check if april tag file exists
        # april_tag_path = os.path.join(calib_dir, self.config.april_tag_filename)
        # if not os.path.isfile(april_tag_path):
        #     logger.error(f"April Tag config file {april_tag_path} does not exist")
        #     return False

        # Check if SMapper repository exists
        if not os.path.isdir(smapper_dir):
            logger.error(f"SMapper directory {smapper_dir} does not exist")
            return False

        return True
