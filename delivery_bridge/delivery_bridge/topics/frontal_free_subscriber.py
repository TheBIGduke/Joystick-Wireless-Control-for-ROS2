#!/usr/bin/env python
import time
import logging

# Third apps
from std_msgs.msg import Float32

# Models
# Serializers
# Local apps
from delivery_bridge.topics.base_topics import BaseSubscriber
from delivery_bridge.webapp.socket_io import emitEvent

logger = logging.getLogger("ros2_log")


class FrontalFreeSubscriber(BaseSubscriber):
    def __init__(
        self,
        node,
        topic_name: str,
        message_type: str,
        distance_safe: float,
        max_rate: int = -1,
    ):
        super().__init__(node, topic_name, message_type, max_rate)

        # attributes for frontal_free topic
        self.distance_safe = distance_safe
        self.frontal_free_available = False
        self.distance = 0.0
        self.notification_alert = False

        # assign message class
        self.message_class = Float32

    # override on subscribed
    def on_subscribed(self):
        self.frontal_free_available = False

    # override on unsubscribed
    def on_unsubscribed(self):
        pass

    # def on_start_low(self):
    #     pass

    # def to_dict(self, msg) -> dict:
    #     return {"distance": round(self.distance, 2), "status": msg}

    # override safe_callback
    def safe_callback(self, msg: Float32):
        # self.node.logger.info(f"received from '{self.topic_name}' topic")
        self.distance = msg.data
        self.frontal_free_available = True
        # self.node.logger.info(f"scan_cb: {self.distance}")

        # if self.distance < self.distance_safe and self.distance > 0.1 and not self.notification_alert:
        #     emitEvent("frontal_free_lane", {"msg": "robot stopped"})
        #     self.notification_alert = True
        #     self.node.logger.info(f"frontal_free_lane, dist: {self.distance}, msg: stopped")
        # if self.distance >= self.distance_safe and self.distance > 0.1 and self.notification_alert:
        #     emitEvent("frontal_free_lane", {"msg": "robot released"})
        #     self.notification_alert = False
        #     self.node.logger.info(f"frontal_free_lane, dist: {self.distance}, msg: released")



        

