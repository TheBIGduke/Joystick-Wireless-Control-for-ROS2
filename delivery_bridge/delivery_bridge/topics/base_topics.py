import time

# Third apps
from rclpy.node import Node
from rclpy.qos import QoSProfile

# local apps


class BaseTopic:
    def __init__(
        self, node: Node, topic_name: str, message_type: str, mode: str = "publisher"
    ):
        self.node = node
        self.topic_name = topic_name
        self.message_type = message_type
        self.message_class = None


class BasePublisher(BaseTopic):
    def __init__(self, node: Node, topic_name: str, message_type: str):
        super().__init__(node, topic_name, message_type, "publisher")
        self.publisher = None

    def try_create_publisher(self):
        # return if already unsubscribed
        if self.publisher is not None:
            return False
        if self.message_class is not None:
            self.publisher = self.node.create_publisher(
                self.message_class, self.topic_name, 1
            )
            self.pub_data = self.message_class()
        else:
            self.node.logger.warning("Unknown topic type: {}".format(self.message_type))
            return False
        self.node.logger.info(
            "Created publisher for topic: {}({})".format(
                self.topic_name, self.message_type
            )
        )
        return True


class BaseSubscriber(BaseTopic):
    def __init__(
        self,
        node: Node,
        topic_name: str,
        message_type: str,
        max_rate: int = -1,
        qos_profile=1,
    ):
        super().__init__(node, topic_name, message_type, "subscriber")
        self.max_rate = max_rate
        self.qos_profile = qos_profile
        self.subscriber = None
        self.already_on_callback = False
        self.send_continuously = False
        self.t0_last_send = 0

    def callback(self, msg):
        # Return if already on callback
        if self.already_on_callback:
            return
        # Return if not enough time has passed since last send
        if self.max_rate > 0 and (time.time() - self.t0_last_send) < (
            1.0 / self.max_rate
        ):
            return
        self.already_on_callback = True
        self.safe_callback(msg)
        self.t0_last_send = time.time()
        self.already_on_callback = False

    def safe_callback(self, msg):
        pass

    def on_subscribed(self):
        pass

    def try_subscribe(self):
        # return if already subscribed
        if self.subscriber is not None:
            return False
        if self.message_class is not None:
            self.subscriber = self.node.create_subscription(
                self.message_class,
                self.topic_name,
                self.callback,
                self.qos_profile,
            )
        else:
            self.node.logger.error(
                f"message_type '{self.message_type}' is not supported."
            )
            return False
        self.on_subscribed()
        if isinstance(self.qos_profile, QoSProfile):
            """
            QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                lifespan=0 nanoseconds,
                deadline=0 nanoseconds,
                liveliness=LivelinessPolicy.SYSTEM_DEFAULT,
                liveliness_lease_duration=0 nanoseconds,
                avoid_ros_namespace_conventions=False
            )
            """
            qos_profile = self.qos_profile.reliability.name
        else:
            qos_profile = self.qos_profile
        self.node.logger.info(
            f"Subscribed to '{self.topic_name}' topic with qos='{qos_profile}'"
        )
        return True

    def on_unsubscribed(self):
        pass

    def try_unsubscribe(self):
        # return if already unsuscribed
        if self.subscriber is None:
            return False

        if self.node.destroy_subscription(self.subscriber):
            self.subscriber = None
            self.on_unsubscribed()
            self.node.logger.info(f"unsubcribed from '{self.topic_name}' topic")
            return True
        else:
            self.node.logger.warning(f"fail unsubscribe to '{self.topic_name}' topic")
            return False
