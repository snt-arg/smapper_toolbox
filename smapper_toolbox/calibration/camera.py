"""
Camera calibration routines for the SMapper toolbox.

This module provides the CameraCalibration class and related utilities to perform camera calibration using the Kalibr toolbox in a Docker environment.
"""

import os
from typing import List

from smapper_toolbox.calibration.helpers import move_kalibr_results
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
        self, bag_name: str, topics: List[str]
    ) -> DockerJobConfig:
        """Generate Docker job configuration for camera calibration.

        Args:
            bag_name: Name of the ROS bag file containing camera data.
            topics: List of ROS topics containing camera images.

        Returns:
            dict: Docker job configuration containing:
                - img_tag: Docker image tag
                - command: Command to run in container
                - env_var: Environment variables
                - volumes: Volume mounts
        """
        target_path = (
            self.config.get_target_path(self.config.calibration.camera.target)
        ) or ""

        rel_target_path = os.path.relpath(
            target_path,
            self.config.workspace.calibration_dir,
        )

        target = os.path.join(self.docker_calibration_dir, rel_target_path)

        # If rolling shutter, add -rs to the end of camera model
        camera_model = self.config.calibration.camera.camera_model

        # fmt: off
        cmd = [
            "rosrun", "kalibr", "kalibr_calibrate_cameras",
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
                f"{self.config.workspace.calibration_dir}:{self.docker_calibration_dir}",
                f"{self.rosbags_dir}:{self.docker_rosbags_dir}",
            ],
        )

        return job_config

    def run(self, **kwargs) -> None:
        """Run the camera calibration process.

        This method:
        1. Finds all calibration bags with camera data
        2. Runs Kalibr calibration for each bag in parallel
        3. Moves generated calibration files to the save directory
        """
        del kwargs  # suppressing linter warning

        logger.info("Running camera calibrations.")
        jobs = []

        bags = self.bag_analyzer.find_calibration_bags(
            self.config.workspace.rosbags_dir, CalibrationMode.CAMERA_ONLY
        )

        logger.info("The following bags will be used")
        for bag in bags:
            print(bag)
            jobs.append(
                self.generate_docker_job_config(
                    bag.name,
                    self.topics_selector.select_camera_topics(bag),
                )
            )

        execute_docker_pool(
            self.docker_helper,
            jobs,
            "Running calibrations in Kalibr...",
            self.config.performance.parallel_calibrations,
        )

        save_dir = os.path.join(
            self.config.workspace.calibration_dir,
            self.config.calibration.camera.save_dir,
        )

        if not os.path.exists(save_dir):
            os.makedirs(save_dir)

        logger.info(f"Moving kalibr results into {save_dir}")
        move_kalibr_results(
            bags,
            self.rosbags_dir,
            save_dir,
            self.config.calibration.camera.output_formats,
        )
