#!/usr/bin/env python

# Python imports
import time

# Third apps
from rclpy.node import Node

# Local apps
from .base_topics import BaseSubscriber


class CmdVelNavSubscriber(BaseSubscriber):
    def __init__(
        self, node: Node, topic_name: str, message_type: str, max_rate: int = -1
    ):
        super().__init__(node, topic_name, message_type, max_rate)

        # attributes for pose topic
        # state for info topic
        self.data_available = False
        self.linear_x = 0.0
        self.angular_z = 0.0

        # assign message class
        self.assign_message_class()

    def assign_message_class(self):
        if self.message_type == "geometry_msgs/Twist":
            from geometry_msgs.msg import Twist
            self.message_class = Twist

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
        self.data_available = False

    # override on unsubscribed
    def on_unsubscribed(self):
        self.data_available = False

    # override safe_callback
    def safe_callback(self, msg):
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z
        self.data_available = True
        # self.node.logger.info(f"Nav v: {self.linear_x}, w: {self.angular_z}")
