import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class MyNode(Node):
    def __init__(self):
        super().__init__('skbot_motion_scan')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT,
            depth=1
        )
        self.sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.__scan_handler,
            qos_profile
        )

    def __scan_handler(self, msg: LaserScan):
        self.get_logger().info(msg.header.frame_id)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()