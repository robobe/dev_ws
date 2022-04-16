from datetime import datetime
import threading
from threading import Event
import rclpy
import time
from rclpy.node import Node
from .connections.mavlink_connection import MavlinkConnection
from rosmav_msgs.msg import Attitude
from rosmav_msgs.srv import CommandBool
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class Vehicle(Node):
    def __init__(self):
        super().__init__('vehicle')
        self.group1 = MutuallyExclusiveCallbackGroup()
        self.srv = self.create_service(CommandBool, '/rosmav/arm', self.arm_callback, callback_group=self.group1)
        self.srv = self.create_service(CommandBool, 'test', self.test_callback, callback_group=self.group1)
        self.__vehicle = MavlinkConnection()
        self.__vehicle.start()
        self.__sync_action_event = Event()
        self.__attitude_publisher_ = self.create_publisher(Attitude, "attitude", 10)
        self.__register_mavlink_messages()
        self.get_logger().info("Start mavlink node")
        # self.create_timer(0.2, self.timer_callback)
    
    # def timer_callback(self):
    #     self.get_logger().info(threading.current_thread().name)
    #     self.get_logger().info("Hello ROS2")

    def __register_mavlink_messages(self):
        self.__vehicle.add_message_listener("HEARTBEAT", self.__heartbeat_cb)
        self.__vehicle.add_message_listener("ATTITUDE", self.__attitude_cb)
        self.__vehicle.add_message_listener("COMMAND_ACK", self.__command_ack_cb)

    def __attitude_cb(self, name, message):
        msg = Attitude()
        msg.pitch = message.pitch
        msg.roll = message.roll
        msg.yaw = message.yaw
        self.__attitude_publisher_.publish(msg)
        # self.get_logger().info(name)
        # self.get_logger().info(str(msg))

    def __command_ack_cb(self, name, message):
        self.get_logger().info(str(message.to_dict()))

    def __heartbeat_cb(self, name, message):
        # self.get_logger().info(threading.current_thread().name)
        from pymavlink.dialects.v20 import ardupilotmega
        is_armed = (message.base_mode & ardupilotmega.MAV_MODE_FLAG_SAFETY_ARMED)
        if is_armed:
            self.__sync_action_event.set()

        self.get_logger().info(str(is_armed))

    def __run_t(self):
        self.get_logger().info("run timer")
        time.sleep(3)
        self.get_logger().info("run timer1")
        self.__notify.set()
        self.__notify.clear()

    def test_callback(self, request: CommandBool.Request, response: CommandBool.Response):
        t = threading.Thread(target=self.__run_t, daemon=True)
        t.start()
        self.get_logger().info(threading.current_thread().name)
        self.get_logger().info("start")
        is_timeout = self.__notify.wait(timeout=2)
        self.get_logger().info("stop")
        
        self.get_logger().info("cleared")
        response.success = is_timeout
        return response

    def arm_callback(self, request: CommandBool.Request, response: CommandBool.Response):
        self.get_logger().info(threading.current_thread().name)
        self.__sync_action_event.clear()
        is_set = self.__sync_action_event.is_set()
        self.get_logger().info(f"sync_action_event is set: {is_set}")
        try:
            if request:
                self.get_logger().info("Try to ARM")
                self.__vehicle.arm()
                is_action_ok = self.__sync_action_event.wait(timeout=3)
                self.get_logger().info("ARMED")
            else:
                self.get_logger().info("Try to DISARM")
                self.__vehicle.disarm()
                is_action_ok = True
            response.success = is_action_ok
        except:
            response.success = False
            self.get_logger().error("Failed to ARMED")
        finally:
            self.__sync_action_event.clear()
        return response


def main(args=None):
    rclpy.init(args=args)
    vehicle_node = Vehicle()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(vehicle_node)

    try:
        # pause the program execution, waits for a request to kill the node (ctrl+c)
        executor.spin()
    finally:
        executor.shutdown()
        vehicle_node.destroy_node()

if __name__ == '__main__':
    main()