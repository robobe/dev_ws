import os
import queue
import threading
import time
from pymavlink import mavutil
from . import connection

os.environ['MAVLINK20'] = '1'


class MavlinkConnection(connection.Connection):
    def __init__(self, device="tcp:127.0.0.1:5760"):
        super().__init__()

        if not device:
            return
        while True:
            try:
                self._master = mavutil.mavlink_connection(device)
                break
            except Exception as e:
                print("Retrying connection in 1 second ...")
                time.sleep(1)

        self._out_msg_queue = queue.Queue()  # a queue for sending data between threads
        self._read_handle = threading.Thread(target=self.dispatch_loop)
        self._read_handle.daemon = True
        self._read_handle.setName("mav_in")
        self._running = True
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 2)

    def start(self):
        self._read_handle.start()

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

    def dispatch_loop(self):
        while self._running:
            msg = self.wait_for_message()
            if not msg:
                time.sleep(0.01)
                continue
            self.notify_message_listeners(msg.get_type(), msg)
            # print(msg.to_dict())

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

