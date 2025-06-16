import os
from typing import List

from smapper_toolbox.logger import logger
from smapper_toolbox.utils.executor import execute_pool


class RosbagsConverter:
    def __init__(self, rosbags_dir: str, parallel_jobs: int = 1):
        self.rosbags_dir = rosbags_dir
        self.ros1_bags_dir = os.path.join(rosbags_dir, "ros1")
        self.ros2_bags_dir = os.path.join(rosbags_dir, "ros2")
        self.parallel_jobs = parallel_jobs

    def convert(self) -> bool:
        if not self._validate_rosbags_dir():
            return False

        logger.info("Searching for ros2 bags to be converted...")

        cmds = []

        for bag in os.listdir(self.ros2_bags_dir):
            src = os.path.join(self.ros2_bags_dir, bag)
            dest = os.path.join(self.ros1_bags_dir, f"{bag}.bag")
            if os.path.isfile(dest):
                logger.debug(f"{bag} has already been converted.")
                continue
            logger.info(f"Found ros2 bag {bag}")
            cmds.append(self._build_cmd(src, dest))

        return execute_pool(
            cmds, f"Converting {len(cmds)} ros2 bags", self.parallel_jobs
        )

    def _validate_rosbags_dir(self) -> bool:
        # Check if rosbags directory exists, and structure is correct
        # rosbags_dir/
        #   |- ros1/
        #   |- ros2/

        if not os.path.isdir(self.rosbags_dir):
            logger.error(f"Rosbags {self.rosbags_dir} directory does not exist")
            return False

        if not os.path.isdir(self.ros2_bags_dir):
            logger.error(
                f"""Ros2 bags {self.ros2_bags_dir} directory does not exist. 
                Make sure you place ros2 bags inside {self.ros2_bags_dir}"""
            )
            return False

        if not os.path.isdir(self.ros1_bags_dir):
            logger.info(
                f"Ros1 bags {self.ros1_bags_dir} directory does not yet exist. Creating it."
            )
            os.makedirs(self.ros1_bags_dir)

        return True

    def _build_cmd(self, src: str, dest: str) -> List[str]:
        return ["rosbags-convert", "--src", src, "--dst", dest]
