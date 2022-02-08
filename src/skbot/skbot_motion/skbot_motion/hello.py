import rclpy
from rclpy.node import Node
class MyNode(Node):
    def __init__(self):
        super().__init__('skbot_motion')
        self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("Hello skbot")

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()