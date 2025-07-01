import os
from abc import ABC, abstractmethod

from smapper_toolbox.config import Config
from smapper_toolbox.rosbags.analyzer import (
    RosbagAnalyzer,
    TopicSelector,
)
from smapper_toolbox.utils import DockerRunner

IMU_NOISE_FILENAME = "imu_noise.yaml"


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
        self.docker_calibration_dir = data_path
        self.docker_rosbags_dir = bags_path
        self.bag_analyzer = bag_analyzer
        self.topics_selector = TopicSelector()

        # We only care about ros1 bags for the calibrators
        self.rosbags_dir = os.path.join(self.config.workspace.rosbags_dir, "ros1")

    @abstractmethod
    def run(self, **kwargs) -> None:
        """Execute the calibration procedure.

        This method must be implemented by concrete calibration classes
        to perform their specific calibration procedure.
        """
        pass
