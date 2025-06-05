import os
from pydantic import BaseModel
import yaml
from typing import Any, Dict, List, Optional

from smapper_toolbox.logger import logger


def read_ros2bag_metadata(path: str) -> Optional[Dict[str, Any]]:
    data = dict()

    if not os.path.isdir(path):
        logger.error("Passed ros2 bag path is not a valid path or directory")
        return None

    metadata_file = os.path.join(path, "metadata.yaml")

    if not os.path.isfile(metadata_file):
        logger.error("Passed ros2 bag does not container a metadata.yaml file")
        return None

    with open(metadata_file, "r") as f:
        data = yaml.safe_load(f)

    return data


class TopicMetadata(BaseModel):
    topic_name: str
    topic_type: str


def read_ros2bag_topics(path: str) -> List[TopicMetadata]:
    topics = []
    metadata = read_ros2bag_metadata(path)

    if metadata is None:
        return []

    topics_metadata = metadata["rosbag2_bagfile_information"][
        "topics_with_message_count"
    ]

    for topic in topics_metadata:
        topic_metadata = topic["topic_metadata"]
        topics.append(
            TopicMetadata(
                topic_name=topic_metadata["name"], topic_type=topic_metadata["type"]
            )
        )

    return topics
