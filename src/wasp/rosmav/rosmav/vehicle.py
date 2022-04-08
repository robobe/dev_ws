import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import time
from .connections.mavlink_connection import MavlinkConnection
from rosmav_msgs.msg import Attitude


class Vehicle(Node):
    def __init__(self):
        super().__init__('vehicle')
        self.srv = self.create_service(SetBool, 'echo_service', self.echo_callback)
        self.__vehicle = MavlinkConnection()
        self.__vehicle.start()
        self.__vehicle.add_message_listener("ATTITUDE", self.__attitude_cb)
        self.__attitude_publisher_ = self.create_publisher(Attitude, "attitude", 10)
        self.get_logger().info("Start service")

    def __attitude_cb(self, name, msg):
        msg = Attitude()
        self.__attitude_publisher_.publish(msg)
        self.get_logger().info(name)
        self.get_logger().info(str(msg))

    def echo_callback(self, request, response: SetBool.Response):
        self.get_logger().info(str(type(request)))
        self.get_logger().info(str(type(response)))
        self.get_logger().info("Incoming request")
        response.success = True
        response.message = "success"
        time.sleep(5)
        return response


def main(args=None):
    rclpy.init(args=args)
    minimal_service = Vehicle()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()