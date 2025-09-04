#!/usr/bin/env python
import logging

# Third apps
# import cv2 as cv
# import numpy as np
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import (
    Pose,
    PoseWithCovariance,
)

# Models
from delivery_bridge.webapp.apps.waypoints.models import Waypoint

# Serializers
# Local apps
from .base_topics import BasePublisher
from delivery_bridge.utils import quaternion_from_euler
from delivery_bridge.webapp.settings import settings

logger = logging.getLogger("ros2_log")


class PosePublisher(BasePublisher):
    def __init__(self, node: Node, topic_name: str, message_type: str):
        super().__init__(node, topic_name, message_type)
        
        # assign message class
        if self.message_type == "geometry_msgs/PoseWithCovarianceStamped":
            from geometry_msgs.msg import PoseWithCovarianceStamped

            self.message_class = PoseWithCovarianceStamped
            self.pub_data = PoseWithCovarianceStamped()

    def set_pose(
        self, position_x: float, position_y: float, orientation_z: float
    ) -> tuple[bool, str]:
        if self.publisher is None:
            return False, "Can't publish to pose topic, not connected"

        self.pub_data.header = Header()
        self.pub_data.header.stamp = self.node.get_clock().now().to_msg()
        self.pub_data.header.frame_id = "map"
        self.pub_data.pose = PoseWithCovariance()
        self.pub_data.pose.covariance = settings.INITIAL_POSE.COVARIANCE
        self.pub_data.pose.pose = Pose()
        self.pub_data.pose.pose.position.x = position_x
        self.pub_data.pose.pose.position.y = position_y
        self.pub_data.pose.pose.position.z = 0.0
        # from euler to quaternion
        qt = quaternion_from_euler(0, 0, orientation_z)
        self.pub_data.pose.pose.orientation.x = qt[0]
        self.pub_data.pose.pose.orientation.y = qt[1]
        self.pub_data.pose.pose.orientation.z = qt[2]
        self.pub_data.pose.pose.orientation.w = qt[3]

        try:
            self.publisher.publish(self.pub_data)
            logger.info("Published to pose topic")
            return True, "Pose published"
        except Exception as e:
            logger.error("Can't publish to pose topic: {}".format(e))
            return False, "Can't publish to pose topic: {}".format(e)

    def set_pose_to_waypoint(self, waypoint: Waypoint) -> tuple[bool, str]:
        return self.set_pose(
            waypoint.position_x, waypoint.position_y, waypoint.orientation
        )
