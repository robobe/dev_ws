import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__("minimal_subscriber")

        # The node subscribes to messages of type std_msgs/String,
        # over a topic named: /minimal
        # The callback function is called as soon as a message is received.
        # The maximum number of queued messages is 10.
        self.subscription = self.create_subscription(
            String, "minimal", self.__sub_callback, 10
        )

    def __sub_callback(self, msg):
        self.get_logger().info(f"I heard: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
