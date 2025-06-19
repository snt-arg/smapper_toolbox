import os
import shutil
from typing import List

from smapper_toolbox.logger import logger
from smapper_toolbox.rosbags.analyzer import (
    CalibrationMode,
)
from smapper_toolbox.utils.executor import DockerJobConfig, execute_docker_pool

from . import CalibrationBase


class CameraCalibration(CalibrationBase):
    """Handles camera calibration using Kalibr using docker environment.

    This class implements camera calibration for one or more cameras
    using the Kalibr toolbox. It processes ROS bags with camera data
    and generates calibration files.

    The calibration process:
    1. Finds all calibration bags with camera data
    2. Runs Kalibr calibration for each bag
    3. Moves generated calibration files to the appropriate directory
    """

    def generate_docker_job_config(
        self, bag_name: str, topics: List[str], rolling_shutter: bool = False
    ) -> DockerJobConfig:
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
        executable = (
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
            "rosrun", "kalibr", executable,
            "--bag", f"/bags/{bag_name}", "--bag-freq", "10",
            "--target", target,
            "--dont-show-report",
        ]
        # fmt: on

        cmd.append("--topics")
        cmd.extend(topics)

        cmd.append("--models")
        cmd.extend([camera_model for _ in range(len(topics))])

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
        jobs = []

        bags = self.bag_analyzer.find_calibration_bags(
            self.config.rosbags_dir, CalibrationMode.CAMERA_ONLY
        )

        logger.info("The following bags will be used")
        for bag in bags:
            logger.info(f"Processing bag: {bag}")
            jobs.append(
                self.generate_docker_job_config(
                    bag.name, self.topics_selector.select_camera_topics(bag)
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
