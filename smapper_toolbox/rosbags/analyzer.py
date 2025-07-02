"""
ROS bag analysis utilities.

This module provides classes and functions to analyze ROS bag files, extract topic information, and assist in selecting topics for calibration workflows.
"""

import hashlib
import json
import os
from enum import Enum
from glob import glob
from pathlib import Path
from typing import List, Optional

from pydantic import BaseModel
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
    def topic_in_bag(bag_info: RosbagInfo, name: str) -> bool:
        for topic in bag_info.camera_topics:
            if topic.name == name:
                return True

        for topic in bag_info.imu_topics:
            if topic.name == name:
                return True

        return False

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


class BagMetadataCache:
    """Cache system for bag metadata to avoid re-analyzing large ROS1 bags."""

    def __init__(self, cache_dir: str = ".bag_cache"):
        self.cache_dir = Path(cache_dir)
        self.cache_dir.mkdir(exist_ok=True)

    def _get_bag_hash(self, bag_path: str) -> str:
        """Generate hash based on bag file size and modification time"""
        try:
            stat = Path(bag_path).stat()
            return hashlib.md5(
                f"{bag_path}_{stat.st_size}_{stat.st_mtime}".encode()
            ).hexdigest()
        except OSError:
            # If file doesn't exist or can't be accessed, return hash of path only
            return hashlib.md5(bag_path.encode()).hexdigest()

    def get_cached_metadata(self, bag_path: str) -> Optional[RosbagInfo]:
        """Retrieve cached metadata if available and valid"""
        cache_file = self.cache_dir / f"{self._get_bag_hash(bag_path)}.json"

        if cache_file.exists():
            try:
                with open(cache_file, "r") as f:
                    data = json.load(f)
                return self._rosbag_info_from_dict(data)
            except Exception as e:
                logger.warning(f"Failed to load cache for {bag_path}: {e}")
                # Remove corrupted cache file
                try:
                    cache_file.unlink()
                except Exception:
                    pass
        return None

    def cache_metadata(self, bag_path: str, bag_info: RosbagInfo):
        """Cache bag metadata"""
        cache_file = self.cache_dir / f"{self._get_bag_hash(bag_path)}.json"

        try:
            data = self._rosbag_info_to_dict(bag_info)
            with open(cache_file, "w") as f:
                json.dump(data, f, indent=2)
            logger.debug(f"Cached metadata for {os.path.basename(bag_path)}")
        except Exception as e:
            logger.warning(f"Failed to cache metadata for {bag_path}: {e}")

    def _rosbag_info_to_dict(self, bag_info: RosbagInfo) -> dict:
        """Convert RosbagInfo to dictionary for JSON serialization"""
        return {
            "version": bag_info.version.value
            if hasattr(bag_info.version, "value")
            else bag_info.version.value,
            "name": bag_info.name,
            "path": bag_info.path,
            "duration": bag_info.duration,
            "topics": [
                {
                    "name": topic.name,
                    "msg_type": topic.msg_type,
                    "msg_count": topic.msg_count,
                    "frequency": topic.frequency,
                }
                for topic in bag_info.topics
            ],
        }

    def _rosbag_info_from_dict(self, data: dict) -> RosbagInfo:
        """Reconstruct RosbagInfo from dictionary"""
        topics = [
            TopicInfo(
                name=topic_data["name"],
                msg_type=topic_data["msg_type"],
                msg_count=topic_data["msg_count"],
                frequency=topic_data["frequency"],
            )
            for topic_data in data["topics"]
        ]

        return RosbagInfo(
            version=RosbagVersion(data["version"]),
            name=data["name"],
            path=data["path"],
            topics=topics,
            duration=data["duration"],
        )

    def clear_cache(self):
        """Clear all cached metadata"""
        for cache_file in self.cache_dir.glob("*.json"):
            try:
                cache_file.unlink()
            except Exception:
                pass
        logger.info("Cache cleared")

    def get_cache_stats(self) -> dict:
        """Get cache statistics"""
        cache_files = list(self.cache_dir.glob("*.json"))
        total_size = sum(f.stat().st_size for f in cache_files)
        return {
            "files": len(cache_files),
            "total_size_mb": round(total_size / (1024 * 1024), 2),
        }


class RosbagAnalyzer:
    """Analyzes ROS bags to extract topic information."""

    def __init__(self, enable_cache: bool = True, cache_dir: str = ".bag_cache"):
        self.cache = BagMetadataCache(cache_dir) if enable_cache else None

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
        Priority: Cache -> ROS2 -> ROS1
        Args:
            bags_dir: Directory containing ROS bags
            mode: Type of calibration to find bags for
        Returns:
            List of suitable BagInfo objects
        """
        ros1_bags_dir = os.path.join(bags_dir, "ros1")
        ros2_bags_dir = os.path.join(bags_dir, "ros2")

        logger.info(f"Analyzing rosbags in {bags_dir}...")

        if not os.path.exists(ros1_bags_dir):
            logger.error(f"ROS1 bags directory not found: {ros1_bags_dir}")
            return []

        suitable_bags = []
        ros2_exists = os.path.exists(ros2_bags_dir)

        # Show analysis strategy
        if self.cache:
            logger.info("Analysis strategy: Cache -> ROS2 -> ROS1")
        elif ros2_exists:
            logger.info("Analysis strategy: ROS2 -> ROS1 (no cache)")
        else:
            logger.info("Analysis strategy: ROS1 only (no cache, no ROS2)")

        for filename in os.listdir(ros1_bags_dir):
            if not filename.endswith(".bag"):
                continue

            bag_path = os.path.join(ros1_bags_dir, filename)
            basename = filename.split(".")[0]

            # Strategy 1: Try cache first (fastest)
            if self.cache:
                cached_info = self.cache.get_cached_metadata(bag_path)
                if cached_info is not None:
                    logger.debug(f"Using cached metadata for {filename}")
                    bag_info = cached_info
                    # Check if bag is suitable for the requested calibration mode
                    if self._is_suitable_for_calibration(bag_info, mode):
                        suitable_bags.append(bag_info)
                    continue

            # Strategy 2: Try ROS2 bag if available (faster than ROS1)
            if ros2_exists:
                ros2_matches = glob(os.path.join(ros2_bags_dir, basename))
                if len(ros2_matches) > 0:
                    ros2_bag_path = os.path.join(ros2_bags_dir, basename)
                    try:
                        bag_info = self.analyze_bag(ros2_bag_path)
                        # Override with ROS1 details for pipeline compatibility
                        bag_info.version = RosbagVersion(1)
                        bag_info.name = filename
                        bag_info.path = bag_path
                        logger.debug(f"Analyzed {filename} using ROS2 bag")

                        # Cache the result for future use
                        if self.cache:
                            self.cache.cache_metadata(bag_path, bag_info)

                        # Check if bag is suitable for the requested calibration mode
                        if self._is_suitable_for_calibration(bag_info, mode):
                            suitable_bags.append(bag_info)
                        continue
                    except Exception as e:
                        logger.warning(
                            f"Failed to analyze ROS2 bag for {filename}, falling back to ROS1: {e}"
                        )

            # Strategy 3: Fallback to ROS1 analysis (slowest)
            logger.debug(f"Analyzing {filename} using ROS1 bag (not cached)")
            try:
                bag_info = self.analyze_bag(bag_path)

                # Cache the result for future use
                if self.cache:
                    self.cache.cache_metadata(bag_path, bag_info)

                # Check if bag is suitable for the requested calibration mode
                if self._is_suitable_for_calibration(bag_info, mode):
                    suitable_bags.append(bag_info)

            except Exception as e:
                logger.error(f"Failed to analyze bag {filename}: {e}")
                continue

        if not suitable_bags:
            logger.warning(f"No suitable bags found for {mode} calibration")
        else:
            logger.info(
                f"Found {len(suitable_bags)} suitable bags for {mode} calibration"
            )

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

    def clear_cache(self):
        """Clear the metadata cache"""
        if self.cache:
            self.cache.clear_cache()

    def get_cache_stats(self) -> dict:
        """Get cache usage statistics"""
        if self.cache:
            return self.cache.get_cache_stats()
        return {"files": 0, "total_size_mb": 0}
