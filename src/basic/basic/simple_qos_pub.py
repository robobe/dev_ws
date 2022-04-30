from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSDurabilityPolicy

qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE)

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import String

class MinimalPubQos(Node):
    def __init__(self) -> None:
        super().__init__("minimal_pub_qos")
        
        self.__pub = self.create_publisher(
            String,
            "topic_system",
            qos_profile_system_default
        )
        self.__timer = self.create_timer(
            timer_period_sec=0.5,
            callback=self.__timer_cb)
        self.__counter = 0

    def __timer_cb(self):
        msg = String()
        msg.data = "Hello QoS {}".format(self.__counter)
        self.__pub.publish(msg)
        self.__counter += 1

def main(args=None):
    rclpy.init(args=args)
    pub_node = MinimalPubQos()
    rclpy.spin(pub_node)
    pub_node.destroy_node()
    rclpy.shutdown()
