from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega
import threading
from threading import Event
import rclpy
import time
from rclpy.node import Node
from .connections.mavlink_connection import MavlinkConnection
from rosmav_msgs.msg import Attitude, Altitude
from rosmav_msgs.srv import CommandBool, CommandString, CommandInt
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from .utils import hypsometric_formula

MAVLINK_ACTION_TIMEOUT = 3
COMMAND_ACK_SUCCESS = 0


class VehicleArmed:
    def __init__(self) -> None:
        self.__state = False
        self.__observers = []

    @property
    def armed(self):
        return self.__state

    @armed.setter
    def armed(self, value):
        if value != self.__state:
            self.__state = value
            for cb in self.__observers:
                # exception handle by the cb
                cb(value)

    def register(self, cb):
        self.__observers.append(cb)


class Vehicle(Node):
    def __init__(self):
        super().__init__("vehicle")
        self.group1 = MutuallyExclusiveCallbackGroup()
        self.vehicle_armed = VehicleArmed()
        self.vehicle_armed.register(self.__on_vehicle_armed)
        self.__altitude_at_arm = 0.0
        self.__current_altitude = None
        self.__vehicle = MavlinkConnection()
        self.__vehicle.start()
        self.__sync_action_event = Event()
        self.__init_comm()
        self.__register_mavlink_messages()
        self.__init_mavlink_message_intervals()
        self.get_logger().info("Start mavlink node")
        # self.create_timer(0.2, self.timer_callback)

    def __init_mavlink_message_intervals(self):
        self.__vehicle.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 2)
        self.__vehicle.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE, 2)

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
        # self.get_logger().info(name)
        # self.get_logger().info(str(msg))

    def __on_vehicle_armed(self, armed):
        if armed:
            self.__altitude_at_arm = self.__current_altitude
            self.__sync_action_event.set()

    def __command_ack_cb(self, name, message):
        """
        COMMAND_ACK ( #77 )
        command	uint16_t	MAV_CMD	Command ID (of acknowledged command).
        result	uint8_t	MAV_RESULT	Result of command.
        """
        self.get_logger().info(str(message.to_dict()))
        if message.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            if message.result == COMMAND_ACK_SUCCESS:
                self.__sync_action_event.set()

    def __heartbeat_cb(self, name, message):
        # self.get_logger().info(threading.current_thread().name)
        self.vehicle_armed.armed = message.base_mode & ardupilotmega.MAV_MODE_FLAG_SAFETY_ARMED

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

    def change_mode_callback(self, request: CommandBool.Request, response: CommandBool.Response):
        try:
            self.__vehicle.mode(request.value)
            is_action_ok = self.__sync_action_event.wait(timeout=MAVLINK_ACTION_TIMEOUT)
            self.__vehicle.set_attitude()
            response.result = is_action_ok
        except:
            response.result = False
        finally:
            self.__sync_action_event.clear()
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
                is_action_ok = self.__sync_action_event.wait(timeout=MAVLINK_ACTION_TIMEOUT)
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


if __name__ == "__main__":
    main()
