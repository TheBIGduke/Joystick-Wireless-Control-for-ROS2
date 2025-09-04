#!/usr/bin/env python

# Third apps
from rclpy.node import Node

# Models
# Serializers
# Local apps
from .base_topics import BasePublisher


class CmdVelPublisher(BasePublisher):
    def __init__(self, node: Node, topic_name: str, message_type: str):
        super().__init__(node, topic_name, message_type)

        # assign message class
        if self.message_type == "geometry_msgs/Twist":
            from geometry_msgs.msg import Twist

            self.message_class = Twist
            self.pub_data = Twist()

    def publish(self, linear_x: float, angular_z: float):
        if self.publisher is None:
            return
        self.pub_data.linear.x = linear_x
        self.pub_data.angular.z = angular_z
        self.publisher.publish(self.pub_data)
