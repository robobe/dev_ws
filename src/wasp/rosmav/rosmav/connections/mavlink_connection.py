import os
os.environ['MAVLINK20'] = '1'

import queue
import threading
import time
import logging
from pymavlink import mavutil
from pymavlink.mavutil import mavfile
from . import connection

logging.basicConfig(format="[%(levelname)s] %(asctime)s %(message)s", level=logging.DEBUG)
logger = logging.getLogger(__name__)

class MavlinkConnection(connection.Connection):
    def __init__(self, device="tcp:127.0.0.1:5760"):
        self.__command_loop_event = threading.Event()
        super().__init__()
        self._send_rate = 10

        if not device:
            return
        while True:
            try:
                self._master: mavfile = mavutil.mavlink_connection(device)
                break
            except Exception as e:
                print("Retrying connection in 1 second ...")
                time.sleep(1)

        self._out_msg_queue = queue.Queue()  # a queue for sending data between threads
        self._read_handle = threading.Thread(target=self.dispatch_loop)
        self._read_handle.daemon = True
        self._read_handle.setName("mav_in")
        self._running = True

        self._in_msg_queue = queue.Queue()  # a queue for sending data between threads
        self._write_handle = threading.Thread(target=self.command_loop)
        self._write_handle.daemon = True
        self._write_handle.setName("mav_out")

        

    def start(self):
        logger.info("Start vehicle")
        self._read_handle.start()
        self._write_handle.start()

    def stop(self):
        self._running = False
        self._master.close()

    def request_message_interval(self, message_id: int, frequency_hz: float):
        """
        Request MAVLink message in a desired frequency,
        documentation for SET_MESSAGE_INTERVAL:
            https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL

        Args:
            message_id (int): MAVLink message ID
            frequency_hz (float): Desired frequency in Hz
        """
        self._master.mav.command_long_send(
            self._master.target_system, self._master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            message_id, # The MAVLink message ID
            1e6 / frequency_hz, # The interval between two messages in microseconds. Set to -1 to disable and 0 to request default rate.
            0, 0, 0, 0, # Unused parameters
            0, # Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
        )

    def send_message(self, msg):
        self._master.mav.send(msg)

    def send_long_command(self, command_type, param1, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0):
        """
        Packs and sends a Mavlink COMMAND_LONG message

        Args:
            command_type: the command type, as defined by MAV_CMD_*
            param1: param1 as defined by the specific command
            param2: param2 as defined by the specific command (default: {0})
            param3: param3 as defined by the specific command (default: {0})
            param4: param4 as defined by the specific command (default: {0})
            param5: param5 (x) as defined by the specific command (default: {0})
            param6: param6 (y) as defined by the specific command (default: {0})
            param7: param7 (z) as defined by the specific command (default: {0})
        """
        try:
            confirmation = 0  # may want this as an input.... used for repeat messages
            msg = self._master.mav.command_long_encode(
                self._master.target_system, 
                self._master.target_component,
                command_type,
                confirmation, 
                param1,
                param2,
                param3,
                param4,
                param5,
                param6,
                param7)

            self._in_msg_queue.put_nowait(msg)
        except:
            logger.error("Failed to send long command", exc_info=True)

    def dispatch_loop(self):
        """
        Wait for message from vehicle and send notification to register callbacks
        """
        while self._running:
            msg = self.wait_for_message()
            if not msg:
                time.sleep(0.01)
                continue
            # if msg.get_type() not in ["ATTITUDE"]:
            #     logger.info(str(msg.to_dict()))
            self.notify_message_listeners(msg.get_type(), msg)
            # print(msg.to_dict())

    def command_loop(self):
        """
        Send command to vehicle
        """
        while self._running:
            self.__command_loop_event.wait(1/self._send_rate)
            try:
                msg = self._in_msg_queue.get_nowait()
                logger.info("Send message")
                self.send_message(msg)
            except queue.Empty:
                # if there is no msgs in the queue, will just continue
                pass
            else:
                pass
                # TODO: implement high rate msg

            
            
    
    def wait_for_message(self):
        """
        Wait for a new mavlink message calls pymavlink's blocking read function to read
        a next message, blocking for up to a timeout of 1s.

        Returns:
            Mavlink message that was read or `None` if the message was invalid.
        """

        # NOTE: this returns a mavlink message
        # this function should not be called outside of this class!
        msg = self._master.recv_match(blocking=True, timeout=1)
        if msg is None:
            # no message received
            return None
        else:
            if (msg.get_type() == 'BAD_DATA'):
                # no message that is useful
                return None

            # send a heartbeat message back, since this needs to be
            # constantly sent so the autopilot knows this exists
            if msg.get_type() == 'HEARTBEAT':
                # send -> type, autopilot, base mode, custom mode, system status
                outmsg = self._master.mav.heartbeat_encode(mavutil.mavlink.MAV_TYPE_GCS,
                                                           mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                                                           mavutil.mavlink.MAV_STATE_ACTIVE)
                self.send_message(outmsg)

            # pass the message along to be handled by this class
            return msg

    def mode(self, mode:int):
        """
        MAV_CMD_DO_SET_MODE (176 )
        1: Mode	Mode	MAV_MODE
        2: Custom Mode	Custom mode - this is system specific, please refer to the individual autopilot specifications for details.	
        3: Custom Submode	Custom sub mode - this is system specific, please refer to the individual autopilot specifications for details.	
        4	Empty
        """
        self.send_long_command(mavutil.mavlink.MAV_CMD_DO_SET_MODE, 
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode)

    def arm(self):
        logger.warning("start send long command")
        self.send_long_command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        logger.warning("send long command")
        # self._master.motors_armed_wait()

    def disarm(self):
        self.send_long_command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0)