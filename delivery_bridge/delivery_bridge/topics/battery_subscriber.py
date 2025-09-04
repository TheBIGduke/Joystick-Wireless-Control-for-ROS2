#!/usr/bin/env python
import time
import logging

# Third apps
from std_msgs.msg import Float64

# Models
# Serializers
# Local apps
from delivery_bridge.topics.base_topics import BaseSubscriber
from delivery_bridge.webapp.socket_io import emitEvent

logger = logging.getLogger("ros2_log")


class BatteryData:
    def __init__(self):
        self.voltage: float = 0.0
        self.percentage: int = 0

    def update(self, voltage: float, percentage: int):
        self.voltage = voltage
        self.percentage = percentage

    def to_dict(self) -> dict:
        return {"voltage": round(self.voltage, 2), "percentage": self.percentage}


class BatterySubscriber(BaseSubscriber):
    def __init__(
        self,
        node,
        topic_name: str,
        message_type: str,
        percentage_zero_safe: float,
        percentage_full_safe: float,
        percentage_low: float,
        voltage_min: float,
        voltage_max: float,
        max_rate: int = -1,
    ):
        super().__init__(node, topic_name, message_type, max_rate)

        # attributes for battery topic
        self.percentage_zero_safe = percentage_zero_safe
        self.percentage_low = percentage_low
        self.percentage_full_safe = percentage_full_safe
        self.voltage_min = voltage_min
        self.voltage_max = voltage_max

        # state for battery topic
        self.battery: BatteryData = BatteryData()
        self.battery_available = False
        self.battery_counter = 0
        self.low_counter = 0
        self.low_time = 0
        self.is_low = False
        self.voltage = 0.0

        # assign message class
        self.message_class = Float64

    # override on subscribed
    def on_subscribed(self):
        self.battery_available = False

    # override on unsubscribed
    def on_unsubscribed(self):
        pass

    def on_start_low(self):
        pass

    # override safe_callback
    def safe_callback(self, msg: Float64):
        # self.node.logger.info(f"Voltage received from '{self.topic_name}' topic")
        self.voltage = msg.data
        percentage_in = (
            (self.voltage - self.voltage_min)
            * 100
            / (self.voltage_max - self.voltage_min)
        )
        percentage = (
            (percentage_in - self.percentage_zero_safe)
            * 100.0
            / (self.percentage_full_safe - self.percentage_zero_safe)
        )
        if percentage < 0.0:
            percentage = 0.0
        if percentage > 100.0:
            percentage = 100.0

        self.battery.update(self.voltage, int(percentage))
        emitEvent("battery", {"data": self.battery.to_dict()})
        self.battery_available = True

        if percentage < self.percentage_low:
            self.low_counter += 1
        else:
            self.low_counter = 0
        if self.low_counter > 5 and not self.is_low:
            self.low_time = time.time()
            self.is_low = True
            logger.warning("Battery is low")
            self.on_start_low()
        elif self.low_counter == 0 and self.is_low:
            self.is_low = False
            logger.info("Battery is not low")
        

