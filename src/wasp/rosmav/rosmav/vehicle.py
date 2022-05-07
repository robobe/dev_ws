import os
os.environ['MAVLINK20'] = '1'

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega
import threading
from threading import Event
import asyncio
import rclpy
import time
from rclpy.node import Node
from .connections.mavlink_connection import MavlinkConnection
from rosmav_msgs.msg import Attitude, Altitude
from rosmav_msgs.srv import CommandBool, CommandString, CommandInt
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.task import Future
from rclpy.task import Task
from .utils import hypsometric_formula

MAVLINK_ACTION_TIMEOUT = 3
COMMAND_ACK_SUCCESS = 0

class IObserver():
    def __init__(self) -> None:
        self._observers = []

    def register(self, cb):
        self._observers.append(cb)

    def unregister(self, cb):
        self._observers.remove(cb)

class VehicleProperties(IObserver):
    def __init__(self) -> None:
        super().__init__()
        self.__properties = {}

    def update(self, index, value):
        self.__properties[index] = value
        for cb in self._observers:
            cb(index, value)

class VehicleArmed(IObserver):
    def __init__(self) -> None:
        super().__init__()
        self.__state = False
        
    @property
    def armed(self):
        return self.__state

    @armed.setter
    def armed(self, value):
        if value != self.__state:
            self.__state = value
            for cb in self._observers:
                # exception handle by the cb
                cb(value)
                
class VehicleMode(IObserver):
    def __init__(self) -> None:
        super().__init__()
        self.__mode = -1
    @property
    def mode(self):
        return self.__mode

    @mode.setter
    def mode(self, value):
        if value != self.__mode:
            self.__mode = value
            for cb in self._observers:
                # exception handle by the cb
                cb(value)

class Vehicle(Node):
    def __init__(self):
        super().__init__("vehicle")
        self.group1 = MutuallyExclusiveCallbackGroup()
        self.vehicle_armed = VehicleArmed()
        self.vehicle_properties = VehicleProperties()
        self.vehicle_mode = VehicleMode()
        self.vehicle_armed.register(self.__on_vehicle_armed)
        self.__altitude_at_arm = 0.0
        self.__current_altitude = None
        self.__vehicle = MavlinkConnection()
        self.__vehicle.start()
        self.__init_comm()
        self.__register_mavlink_messages()
        self.__init_mavlink_message_intervals()
        self.get_logger().info("Start mavlink node")
        # self.create_timer(3, self.debug_timer_callback)

    async def debug_timer_callback(self):
        """
        debug time to test actions
        """
        self.get_logger().info("time test")
        await self.param_request_async("FRAME_CLASS")
        self.get_logger().info("time action --------------------------")

    # region init
    def __init_mavlink_message_intervals(self):
        self.__vehicle.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 2)
        self.__vehicle.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE, 2)
        self.__vehicle.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_PARAM_VALUE, 2)
        
    def __init_comm(self):
        self.__attitude_publisher_ = self.create_publisher(Attitude, "rosmav/attitude", 10)
        self.__altitude_publisher_ = self.create_publisher(Altitude, "rosmav/altitude", 10)
        self.__arm_service = self.create_service(
            CommandBool, "/rosmav/arm", self.arm_callback, callback_group=self.group1
        )
        self.__mode_service = self.create_service(
            CommandInt,
            "rosmav/change_mode",
            self.change_mode_callback,
            callback_group=self.group1,
        )

    def __register_mavlink_messages(self):
        self.__vehicle.add_message_listener("HEARTBEAT", self.__heartbeat_cb)
        self.__vehicle.add_message_listener("ATTITUDE", self.__attitude_cb)
        self.__vehicle.add_message_listener("COMMAND_ACK", self.__command_ack_cb)
        self.__vehicle.add_message_listener("SCALED_PRESSURE", self.__baro_cb)  # SCALED_PRESSURE ( #29 )
        self.__vehicle.add_message_listener("PARAM_VALUE", self.__param_value_cb) # PARAM_VALUE ( #22)
    # endregion init
    async def param_request_async(self, param_name):
        fut = Future(executor=executor)
        def param_handler(index, value):
            if param_name == index:
                fut.set_result(True)

        self.vehicle_properties.register(param_handler)
        self.__vehicle.param_request(param_name)
        fut.add_done_callback(lambda f: self.vehicle_properties.unregister(param_handler))
        return fut

    # region mavlink meassage handlers 
    def __param_value_cb(self, name, message):
        """
        PARAM_VALUE ( #22)
        param_id
        param_value
        param_type
        param_count
        param_index
        """
        try:
            self.get_logger().info(f"param {message.param_id} value {message.param_value}")
            self.vehicle_properties.update(message.param_id, message.param_value)
            # if "PARAM_VALUE" in self.__futures_cb and message.param_id == "FRAME_CLASS":
                # self.get_logger().info("-------------------------------------aaaaaaaaaaaaa")
                # fu = self.__futures_cb["PARAM_VALUE"]
                # fu.set_result(True)
                # self.get_logger().info("-------------------------------------bbbbbbbbbbbb")
        except BaseException as e:
            print(e)

    def __baro_cb(self, name, message):
        """
        SCALED_PRESSURE ( #29 )
        time_boot_ms	uint32_t	ms	Timestamp (time since system boot).
        press_abs	float	hPa	Absolute pressure
        press_diff	float	hPa	Differential pressure 1
        temperature	int16_t	cdegC	Absolute pressure temperature
        temperature_press_diff **	int16_t	cdegC	Differential pressure temperature (0, if not available). Report values of 0 (or 1) as 1 cdegC.
        """
        h = hypsometric_formula(message.press_abs, message.temperature / 1000)
        self.__current_altitude = h
        data = Altitude()
        data.current_altitude = h
        data.ref_altitude_at_arm = self.__altitude_at_arm
        self.__altitude_publisher_.publish(data)
        self.get_logger().info("Current h = {}".format(h))

    def __attitude_cb(self, name, message):
        msg = Attitude()
        msg.pitch = message.pitch
        msg.roll = message.roll
        msg.yaw = message.yaw
        self.__attitude_publisher_.publish(msg)

    def __command_ack_cb(self, name, message):
        """
        COMMAND_ACK ( #77 )
        command	uint16_t	MAV_CMD	Command ID (of acknowledged command).
        result	uint8_t	MAV_RESULT	Result of command.
        """
        self.get_logger().info(str(message.to_dict()))
        if message.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            if message.result == COMMAND_ACK_SUCCESS:
                self.vehicle_mode.mode = message.result

    def __heartbeat_cb(self, name, message):
        print(message)
        # self.get_logger().info(threading.current_thread().name)
        # set true/false if vehicle armed
        self.vehicle_armed.armed = message.base_mode & ardupilotmega.MAV_MODE_FLAG_SAFETY_ARMED == ardupilotmega.MAV_MODE_FLAG_SAFETY_ARMED

    # endregion mavlink meassage handlers

    def __on_vehicle_armed(self, armed):
        if armed:
            self.__altitude_at_arm = self.__current_altitude
    
    # region system callback    
    def change_mode_callback(self, request: CommandBool.Request, response: CommandBool.Response):
        """
        method call by callgroup1
        """
        mode = request.value

        sync_event = Event()
        sync_event.clear()
        
        def handler():
            sync_event.set()

        self.vehicle_mode.register(handler)
        try:
            self.__vehicle.mode(mode)
            is_action_ok = sync_event.wait(timeout=MAVLINK_ACTION_TIMEOUT)
            response.result = is_action_ok
            return response
        finally:
            self.vehicle_mode.unregister(handler)

    async def arm_callback(self, request: CommandBool.Request, response: CommandBool.Response):
        """
        method call by callgroup1
        """
        arm_request = request.value
        # pre run validation
        if arm_request and self.vehicle_armed.armed:
            # Check if allready ARMED
            response.success = True
            return response

        sync_event = Event()
        sync_event.clear()
        action_ok = False

        def arm_handler(armed):
            self.get_logger().info(f"Vehicle set to arm: {armed}")
            nonlocal action_ok
            action_ok = arm_request == armed
            sync_event.set()

        self.vehicle_armed.register(arm_handler)
        handler = self.__vehicle.arm if arm_request else self.__vehicle.disarm
        try:
            handler()
            is_action_ok = sync_event.wait(timeout=MAVLINK_ACTION_TIMEOUT)
            response.success = is_action_ok and action_ok
            return response
        finally:
            self.vehicle_armed.unregister(arm_handler)
        
    # endregion system callback    

executor = None

def main(args=None):
    rclpy.init(args=args)
    vehicle_node = Vehicle()
    global executor
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
    