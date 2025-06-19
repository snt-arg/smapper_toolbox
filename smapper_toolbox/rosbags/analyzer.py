"""
ROS bag analysis utilities.

This module provides classes and functions to analyze ROS bag files, extract topic information, and assist in selecting topics for calibration workflows.
"""

import os
from enum import Enum
from typing import List, Optional
from pydantic import BaseModel
from pathlib import Path

from rosbags.highlevel import AnyReader

from smapper_toolbox.logger import logger


class RosbagVersion(Enum):
    VERSION_1 = 1
    VERSION_2 = 2


class CalibrationMode(Enum):
    """Enumeration of calibration modes."""

    CAMERA_ONLY = "camera"
    IMU_ONLY = "imu"
    CAMERA_IMU = "camera_imu"


class TopicInfo(BaseModel):
    name: str
    msg_type: Optional[str]
    msg_count: int
    frequency: float


class RosbagInfo(BaseModel):
    version: RosbagVersion
    duration: float
    path: str
    name: str
    topics: List[TopicInfo]

    @property
    def camera_topics(self) -> List[TopicInfo]:
        """Get all camera/image topics from the bag."""
        image_types = ["sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage"]
        return [topic for topic in self.topics if topic.msg_type in image_types]

    @property
    def imu_topics(self) -> List[TopicInfo]:
        """Get all IMU topics from the bag."""
        imu_types = ["sensor_msgs/msg/Imu"]
        return [topic for topic in self.topics if topic.msg_type in imu_types]

    def __str__(self) -> str:
        txt = f"""Bag Info:
          Version: {self.version.value}
          Name: {self.name}
          Duration: {self.duration * 2e-9:.2f} sec
          Topics: {self.topics}
        """

        return txt


class TopicSelector:
    """Handles topic selection logic for different calibration types."""

    @staticmethod
    def select_camera_topics(
        bag_info: RosbagInfo, topic_patterns: Optional[List[str]] = None
    ) -> List[str]:
        """
        Select camera topics for calibration.

        Args:
            bag_info: Information about the ROS bag
            topic_patterns: Optional list of patterns to match topics against

        Returns:
            List of selected topic names
        """
        camera_topics = bag_info.camera_topics

        if not camera_topics:
            logger.warning(f"No camera topics found in bag {bag_info.name}")
            return []

        # If patterns are provided, filter topics
        if topic_patterns:
            selected_topics = []
            for pattern in topic_patterns:
                matching_topics = [
                    topic.name for topic in camera_topics if pattern in topic.name
                ]
                selected_topics.extend(matching_topics)
            return list(set(selected_topics))  # Remove duplicates

        # Default: return all camera topics
        return [topic.name for topic in camera_topics]

    @staticmethod
    def select_imu_topics(
        bag_info: RosbagInfo, topic_patterns: Optional[List[str]] = None
    ) -> List[str]:
        """
        Select IMU topics for calibration.

        Args:
            bag_info: Information about the ROS bag
            topic_patterns: Optional list of patterns to match topics against

        Returns:
            List of selected topic names
        """
        imu_topics = bag_info.imu_topics

        if not imu_topics:
            logger.warning(f"No IMU topics found in bag {bag_info.name}")
            return []

        # If patterns are provided, filter topics
        if topic_patterns:
            selected_topics = []
            for pattern in topic_patterns:
                matching_topics = [
                    topic.name for topic in imu_topics if pattern in topic.name
                ]
                selected_topics.extend(matching_topics)
            return list(set(selected_topics))  # Remove duplicates

        # Default: return the first IMU topic (most common case)
        return [imu_topics[0].name] if imu_topics else []


class RosbagAnalyzer:
    """Analyzes ROS bags to extract topic information."""

    def analyze_bag(self, bag_path: str) -> RosbagInfo:
        """
        Analyze a ROS bag file to extract topic information.

        Args:
            bag_path: Path to the ROS bag file

        Returns:
            BagInfo object containing bag analysis results
        """

        filename = os.path.basename(bag_path)

        with AnyReader([Path(bag_path)]) as reader:
            duration = reader.duration

            topics = [
                TopicInfo(
                    name=key,
                    msg_type=val.msgtype,
                    msg_count=val.msgcount,
                    frequency=round(val.msgcount / (duration * 1e-9)),
                )
                for key, val in reader.topics.items()
            ]

            return RosbagInfo(
                version=RosbagVersion(2 if reader.is2 else 1),
                name=filename,
                path=bag_path,
                topics=topics,
                duration=duration,
            )

    def find_calibration_bags(
        self, bags_dir: str, mode: CalibrationMode
    ) -> List[RosbagInfo]:
        """
        Find suitable bags for calibration based on their content.

        Args:
            bags_dir: Directory containing ROS bags
            mode: Type of calibration to find bags for

        Returns:
            List of suitable BagInfo objects
        """
        ros1_bags_dir = os.path.join(bags_dir, "ros1")
        logger.info(f"Analyzing ros1 bags in {ros1_bags_dir}... (patience)")
        if not os.path.exists(ros1_bags_dir):
            logger.error(f"ROS1 bags directory not found: {ros1_bags_dir}")
            return []

        suitable_bags = []

        for filename in os.listdir(ros1_bags_dir):
            if not filename.endswith(".bag"):
                continue

            bag_path = os.path.join(ros1_bags_dir, filename)
            bag_info = self.analyze_bag(bag_path)

            # Check if bag is suitable for the requested calibration mode
            if self._is_suitable_for_calibration(bag_info, mode):
                suitable_bags.append(bag_info)

        return suitable_bags

    def _is_suitable_for_calibration(
        self, bag_info: RosbagInfo, mode: CalibrationMode
    ) -> bool:
        """
        Check if a bag is suitable for a specific calibration mode.

        Args:
            bag_info: Information about the bag
            mode: Calibration mode to check for

        Returns:
            True if bag is suitable, False otherwise
        """
        if mode == CalibrationMode.CAMERA_ONLY:
            return len(bag_info.camera_topics) > 0
        elif mode == CalibrationMode.IMU_ONLY:
            return len(bag_info.imu_topics) > 0 and bag_info.duration >= 1.08 * 1e13
        elif mode == CalibrationMode.CAMERA_IMU:
            return len(bag_info.camera_topics) > 0 and len(bag_info.imu_topics) > 0
