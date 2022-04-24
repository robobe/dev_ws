from ast import Param
from inspect import Parameter
import re
from rclpy.executors import MultiThreadedExecutor
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from .utils.pid import PID
from rosmav_msgs.msg import Attitude, Altitude
from rosmav_msgs.srv import CommandFloat


class CameraGuide(Node):
    def __init__(self):
        super().__init__("camera_guide")
        self.__init_parameters()
        self.add_on_set_parameters_callback(self.__param_update_cb)
        
        self.__altitude_pid = PID()
        self.__init_comm()
        
        self.__params_pid_action_mappings = self.__get_params_pid_action_mapping()
        
        timer_period = 2
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def __get_params_pid_action_mapping(self):
        mapping = {
            "alt_pid_p": self.__altitude_pid.setKp,
            "alt_pid_i": self.__altitude_pid.setKi
        }
        return mapping

    def timer_callback(self):
        my_param = self.get_parameter('alt_pid_p').get_parameter_value().double_value
        self.get_logger().info("param: {}".format(my_param))

    def __param_update_cb(self, params):
        p: Parameter
        for p in params:
            if p.name in self.__params_pid_action_mappings:
                value = self.get_parameter(p.name).get_parameter_value().double_value
                self.__params_pid_action_mappings[p.name](value)
        result = SetParametersResult()
        result.successful = True
        return result

    def __init_parameters(self):
        self.declare_parameter("alt_pid_p", value=1.0)
        self.declare_parameter("alt_pid_i", value=0.2)

    def __init_comm(self):
        self.__attitude_sub = self.create_subscription(Attitude, "rosmav/attitude", self.__attitude_handler, 10)
        self.__altitude_pub = self.create_publisher(Altitude, "rosmav/altitude", 10)
        self.__alt_setpoint_service = self.create_service(CommandFloat, "/wasp/alt_setpoint", self.alt_setpoint_handler)

    def alt_setpoint_handler(self, request: CommandFloat.Request, response: CommandFloat.Response):
        self.get_logger().info("Request alt setpoint with data: {}".format(request.value))
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