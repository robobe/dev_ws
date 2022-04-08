import threading
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class Talker(Node):
    def __init__(self):
        # Calls Node.__init__('talker')
        super().__init__('talker')
        self.i = 0
        self.pub = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)


    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: {0}'.format(self.i)
        self.i += 1
        t_name = threading.current_thread().name
        self.get_logger().info('Publishing: "{0}" on thread {1} running in process {2}'.format(msg.data, t_name, os.getpid()))
        self.pub.publish(msg)

    # def loop(self):
    #     while True:
    #         self.get_logger().info("inside loop")
    #         time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    try:
        talker = Talker()
        talker.get_logger().info("Start talker")
        rclpy.spin(talker)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()
        talker.destroy_node()


if __name__ == '__main__':
    main()