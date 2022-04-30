# from rclpy.qos import QoSProfile
# from rclpy.qos import QoSReliabilityPolicy
# from rclpy.qos import QoSDurabilityPolicy
# qos_profile = QoSProfile(
#             depth=10,
#             reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
#             durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE)

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

class MinimalSubQoS(Node):
    def __init__(self) -> None:
        super().__init__("minimal_sub_qos")
        
        self.__sub = self.create_subscription(
            String,
            "topic",
            self.__cb,
            qos_profile_sensor_data
        )
        self.__sub

    def __cb(self, msg:String):
        self.get_logger().info(msg.data)


def main(args=None):
    rclpy.init(args=args)
    sub_node = MinimalSubQoS()
    rclpy.spin(sub_node)
    sub_node.destroy_node()
    rclpy.shutdown()