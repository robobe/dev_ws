from rclpy.executors import MultiThreadedExecutor
import rclpy
import time
from rclpy.node import Node
from .utils.pid import PID
from rosmav_msgs.msg import Attitude, Altitude
from rosmav_msgs.srv import CommandFloat

class CameraGuide(Node):
    def __init__(self):
        super().__init__("camera_guide")
        self.__altitude_pid = PID()
        self.__init_comm()

    def __init_comm(self):
        self.__attitude_sub = self.create_subscription(Attitude, "rosmav/attitude", self.__attitude_handler)
        self.__altitude_sub = self.create_publisher(Altitude, "rosmav/altitude", self.__altitude_handler)
        self.__alt_setpoint_service = self.create_service(CommandFloat, "/rosmav/alt_setpoint", self.alt_setpoint_handler, callback_group=self.group1
        )

    def alt_setpoint_handler(self, request: CommandFloat.Request, response: CommandFloat.Response):
        self.__altitude_pid.SetPoint = request.value
        response.success = True
        return response

    def __attitude_handler(self, msg: Attitude):
        pass

    def __altitude_handler(self, msg: Altitude):
        self.__altitude_pid.update(msg.current_altitude)

def main(args=None):
    rclpy.init(args=args)
    vehicle_node = CameraGuide()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(vehicle_node)

    try:
        # pause the program execution, waits for a request to kill the node (ctrl+c)
        executor.spin()
    finally:
        executor.shutdown()
        vehicle_node.destroy_node()


if __name__ == "__main__":
    main()