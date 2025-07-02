"""
Camera-IMU calibration routines for the SMapper toolbox.

This module provides the IMUCameraCalibration class and related utilities to perform camera-IMU calibration using the Kalibr toolbox in a Docker environment.
"""

import os
from glob import glob
from typing import List, Optional

import yaml

from smapper_toolbox.calibration.helpers import move_kalibr_results
from smapper_toolbox.logger import logger
from smapper_toolbox.rosbags.analyzer import (
    CalibrationMode,
)
from smapper_toolbox.utils.executor import DockerJobConfig, execute_docker_pool

from . import IMU_NOISE_FILENAME, CalibrationBase


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

    def generate_docker_job_config(
        self, bag_name: str, camchain_path: str, imu_config_paths: List[str]
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
        camchain_relative = os.path.join(
            self.docker_calibration_dir,
            os.path.relpath(camchain_path, self.config.workspace.calibration_dir),
        )

        imu_config_relative_paths = []
        for path in imu_config_paths:
            imu_config_relative_paths.append(
                os.path.join(
                    self.docker_calibration_dir,
                    os.path.relpath(path, self.config.workspace.calibration_dir),
                )
            )

        target_path = (
            self.config.get_target_path(self.config.calibration.camera_imu.target)
        ) or ""

        rel_target_path = os.path.relpath(
            target_path,
            self.config.workspace.calibration_dir,
        )

        target = os.path.join(self.docker_calibration_dir, rel_target_path)

        # fmt: off
        cmd = [
            "rosrun", "kalibr", "kalibr_calibrate_imu_camera",
            "--bag", f"/bags/{bag_name}",
            "--cams", camchain_relative,
            "--target", target,
            "--reprojection-sigma", str(self.config.calibration.camera_imu.reprojection_sigma),
            "--dont-show-report"
        ]

        cmd.append("--imu")
        cmd.extend(imu_config_relative_paths)

        # NOTE: Assuming all imu models are calibrated.
        cmd.append("--imu-models")
        cmd.extend(["calibrated" for _ in imu_config_paths])
        # fmt: on

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

    def _find_camchain(self, bag_name: str, calib_results_dir: str) -> Optional[str]:
        """Find the camera calibration YAML file for a given bag.

        Args:
            bag_name: Name of the ROS bag file.
            calib_files_path: Path to the calibration files directory.

        Returns:
            Optional[str]: Path to the camera calibration YAML file, or None if not found.
        """
        camchains = [
            file
            for x in os.walk(calib_results_dir)
            for file in glob(os.path.join(x[0], "*camchain.yaml"))
        ]

        for camchain in camchains:
            os.path.basename(camchain)
            bag_filename = bag_name.split(".")[0]
            if bag_filename in camchain:
                return camchain
        return None

    def run(self, **kwargs) -> None:
        """Run the camera-IMU calibration process.

        This method:
        1. Finds all camera-IMU calibration bags
        2. Verifies existence of required calibration files
        3. Runs Kalibr calibration for each bag in parallel
        4. Moves generated calibration files to the static directory
        """
        del kwargs  # suppressing linter warning

        logger.info("Running camera-IMU calibrations.")
        jobs = []

        calib_files_path = os.path.join(
            self.config.workspace.calibration_dir,
            self.config.calibration.camera.save_dir,
        )

        imu_save_dir = os.path.join(
            self.config.workspace.calibration_dir,
            self.config.calibration.imu.save_dir,
        )

        bags = self.bag_analyzer.find_calibration_bags(
            self.config.workspace.rosbags_dir, CalibrationMode.CAMERA_IMU
        )

        for bag in bags:
            camera_yaml = self._find_camchain(bag.name, calib_files_path)

            if camera_yaml is None:
                logger.error(
                    "No camera calibration files found. Please run camera calibration first."
                )
                return

            # BUG: We must go throug the imu.save_dir and get the yamls due to
            # the change made to IMUCalibrator

            imu_configs = []
            for imu_yaml in os.listdir(imu_save_dir):
                config_file = os.path.join(
                    imu_save_dir,
                    imu_yaml,
                    IMU_NOISE_FILENAME,
                )

                if not os.path.exists(config_file):
                    logger.error(
                        "No IMU calibration file found. Please run IMU calibration first."
                    )
                    continue

                with open(config_file, "r") as f:
                    data = yaml.safe_load(f)
                    target_topic = data["rostopic"]
                    if not self.topics_selector.topic_in_bag(bag, target_topic):
                        logger.info(
                            f"Rosbag {bag.name} does not contain IMU topic {target_topic}. Not considering {imu_yaml} config."
                        )
                        continue

                imu_configs.append(config_file)

            if len(imu_configs) == 0:
                logger.error(
                    f"Could not find any valid imu noise for this bag: {bag.name}. Aborting."
                )
                return

            jobs.append(
                self.generate_docker_job_config(bag.name, camera_yaml, imu_configs)
            )

        execute_docker_pool(
            self.docker_helper,
            jobs,
            "Running camera-IMU calibrations in Kalibr...",
            self.config.performance.parallel_calibrations,
        )

        save_dir = os.path.join(
            self.config.workspace.calibration_dir,
            self.config.calibration.camera_imu.save_dir,
        )

        logger.info(f"Moving kalibr results into {save_dir}")
        move_kalibr_results(
            bags,
            self.rosbags_dir,
            save_dir,
            self.config.calibration.camera_imu.output_formats,
        )
