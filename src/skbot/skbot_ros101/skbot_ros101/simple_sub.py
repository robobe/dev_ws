import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.sub = self.create_subscription(String, 'chatter', self.chatter_callback, 10)

    def chatter_callback(self, msg):
        t_name = threading.current_thread().name
        self.get_logger().info(f'I heard: "{msg.data}"  on thread {t_name} runnning in {os.getpid()}')


def main(args=None):
    rclpy.init(args=args)
    try:
        listener = Listener()
        rclpy.spin(listener)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()
        listener.destroy_node()


if __name__ == '__main__':
    # Runs a listener node when this script is run directly (not through an entrypoint)
    main()