#!/usr/bin/env python

# Python imports
import time

# Third apps
from rclpy.node import Node

# Local apps
from .base_topics import BaseSubscriber
from ..utils import euler_from_quaternion
from delivery_bridge.webapp.socket_io import emitEvent


class RobotPoseData:
    def __init__(self):
        self.position_x: float = 0.0
        self.position_y: float = 0.0
        self.orientation: float = 0.0
        self.covariance: list = []
        self.timestamp: float = time.time()

    def update(
        self,
        position_x: float,
        position_y: float,
        orientation: float,
        covariance: list,
    ):
        self.position_x = position_x
        self.position_y = position_y
        self.orientation = orientation
        self.covariance = covariance
        self.timestamp = time.time()

    def to_dict(self) -> dict:
        return {
            "position_x": round(self.position_x, 2),
            "position_y": round(self.position_y, 2),
            "orientation": round(self.orientation, 2),
            "timestamp": self.timestamp,
        }


class PoseSubscriber(BaseSubscriber):
    def __init__(
        self, node: Node, topic_name: str, message_type: str, max_rate: int = -1
    ):
        super().__init__(node, topic_name, message_type, max_rate)

        # attributes for pose topic
        # state for pose topic
        self.pose_data = RobotPoseData()
        self.pose_available = False

        # assign message class
        self.assign_message_class()

    def assign_message_class(self):
        if self.message_type == "geometry_msgs/Pose":
            from geometry_msgs.msg import Pose
            self.message_class = Pose

    def change_topic(self, topic_name: str, message_type: str):
        if self.topic_name == topic_name:
            return
        # unsubscribe from current topic
        self.try_unsubscribe()
        self.node.logger.info(
            f"Changing pose topic from '{self.topic_name}' to '{topic_name}'"
        )
        # change topic name and message type
        self.topic_name = topic_name
        self.message_type = message_type
        # assign message class
        self.assign_message_class()

    # override on subscribed
    def on_subscribed(self):
        self.pose_available = False

    # override on unsubscribed
    def on_unsubscribed(self):
        self.pose_available = False

    # override safe_callback
    def safe_callback(self, msg):
        if self.message_type == "geometry_msgs/Pose":
            position_x = msg.position.x
            position_y = msg.position.y
            orientation = euler_from_quaternion(
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w,
            )[2]
            covariance = []

        self.pose_data.update(position_x, position_y, orientation, covariance)
        self.pose_available = True
        emitEvent("robot_pose", self.pose_data.to_dict())
