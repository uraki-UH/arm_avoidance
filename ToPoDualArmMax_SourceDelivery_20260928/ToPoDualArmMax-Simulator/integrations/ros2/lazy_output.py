"""初回送信時の配信口生成と送信停止後のROSグラフからの解除。"""
import threading
import time

from rclpy.qos import DurabilityPolicy


class lazy_publisher:
    def __init__(self, node, kind, topic, qos, max_idle_sec):
        self.node, self.kind, self.topic, self.qos = node, kind, topic, qos
        self.max_idle_sec = max_idle_sec
        self.enable_idle_expiry = getattr(qos, 'durability', None) != DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = None
        self.last_publish_sec = 0.
        self.lock = threading.RLock()

    def publish(self, message):
        with self.lock:
            if self.publisher is None:
                self.publisher = self.node.create_publisher(self.kind, self.topic, self.qos)
            self.publisher.publish(message)
            self.last_publish_sec = time.monotonic()

    def expire(self, now_sec):
        with self.lock:
            if (self.enable_idle_expiry and self.publisher is not None and
                    now_sec-self.last_publish_sec >= self.max_idle_sec):
                self.close()

    def close(self):
        with self.lock:
            if self.publisher is not None:
                self.node.destroy_publisher(self.publisher)
                self.publisher = None


class output_registry:
    def __init__(self, node, max_idle_sec=3.):
        self.node, self.max_idle_sec = node, max_idle_sec
        self.publishers = []
        self.timer = node.create_timer(.5, self.expire)

    def create_publisher(self, kind, topic, qos):
        publisher = lazy_publisher(self.node, kind, topic, qos, self.max_idle_sec)
        self.publishers.append(publisher)
        return publisher

    def expire(self):
        now_sec = time.monotonic()
        for publisher in self.publishers:
            publisher.expire(now_sec)

    def close(self):
        self.node.destroy_timer(self.timer)
        for publisher in self.publishers:
            publisher.close()
