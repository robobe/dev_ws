#!/usr/bin/env python3
import asyncio
import sys
import struct
import getmac

import rclpy
from rclpy.node import Node
from mk4_msgs.srv import Headlight as HeadlightSrv
from mk4_msgs.msg import Heartbeat
from rclpy.executors import MultiThreadedExecutor

compiled_dsdl_dir = "/home/pi/.yakut"

sys.path.insert(0,str(compiled_dsdl_dir))

import uavcan.node  # noqa
import uavcan.primitive.scalar

from pyuavcan.transport.can.media.socketcan import SocketCANMedia
from pyuavcan.transport.can import CANTransport
from pyuavcan.presentation import Presentation
from pyuavcan.application._node_factory import SimpleNode
from pyuavcan.application._registry_factory import make_registry
from pyuavcan.application import register

class HeadlightUavcanInterface():
    RESPONSE_SUCCESS = 0
    RESPONSE_FAILURE = 1
    RESPONSE_NOT_AUTHERIZED = 2
    RESPONSE_BAD_COMMAND = 3
    RESPONSE_BAD_PARAMETER = 4
    RESPONSE_BAD_STATE = 5
    RESPONSE_INTERNAL_ERROR = 6
    RESPONSE_NO_RESPONSE = -1

    def __init__(self, can_interface, mtu, local_node_id, local_node_name, \
            v_major, v_minor, register_file, reg_bit_key, reg_bit_value, \
            execute_command_identifier, publisher_port_name, reg_pub_key, reg_pub_value):

        mac_addr = getmac.get_mac_address().split(':')
        mac_addr_int  = []
        for x in range(len(mac_addr)):
            mac_addr_int.append(int(mac_addr[x],16)) ##converts string to int

        unique_id_padding = [0] * (16-len(mac_addr_int)) ## 16 is used here as the unique id is at uint8[16] array
        ## According to their documentation the unique id should be globally unique to each node, as we will have
        ## multiple nodes on the ccu, the local node id is added to the unique id for the node. This is added in
        ## with a byte gap from the mac address so if/when debugging it is easy to decipher the two
        unique_id_padding[16-len(mac_addr_int)-2] = local_node_id

        unique_id = unique_id_padding + mac_addr_int
        uavcan_node_info = uavcan.node.GetInfo_1.Response(
            software_version=uavcan.node.Version_1(major=v_major, minor=v_minor),
            name=local_node_name, unique_id=unique_id,protocol_version=uavcan.node.Version_1(major=1, minor=0))

        registry = make_registry(register_file=register_file)
        registry.setdefault(reg_pub_key,register.Value(integer16=register.Integer16(reg_pub_value)))
        registry.setdefault(reg_bit_key, register.Value(string=register.String(reg_bit_value)))
        media = SocketCANMedia(can_interface, mtu=mtu)
        transport = CANTransport(media, local_node_id=local_node_id)
        presentation = Presentation(transport)
        self.headlight_uavcan_node = SimpleNode(presentation,uavcan_node_info, registry)

        self.headlight_activate_publisher = self.headlight_uavcan_node.make_publisher(\
            uavcan.primitive.scalar.Bit_1_0, publisher_port_name)

        self.id_headlightclient_dict = {}
        self.id_name_dict = {}
        self.id_command_dict = {}

        self.send_activate_headlight = False
        self.activate_headlight = False

        self.byte_array_to_send = bytearray()
        self.uavcan_response = None
        self.send_req = False
        self.uavcan_execute_command_identifier = execute_command_identifier
        self.headlight_uavcan_node.start()

    async def async_execute_command(self):
        while 1:
            await asyncio.sleep(0.0001)
            for key in self.id_headlightclient_dict:
                if self.id_headlightclient_dict[key] is None:
                    self.id_headlightclient_dict[key] = self.headlight_uavcan_node.make_client(\
                        uavcan.node.ExecuteCommand_1_1, key, self.id_name_dict[key] + '.headlight')

                if self.id_command_dict[key] is not None and self.send_req:
                    req = uavcan.node.ExecuteCommand_1.Request()
                    req.command= self.uavcan_execute_command_identifier
                    req.parameter = self.id_command_dict[key]
                    attempts = 0
                    while attempts < 3:
                        try:
                            response, metadata = await(self.id_headlightclient_dict[key].call(req))
                            self.uavcan_response = response.status
                            break
                        except:
                            attempts += 1
                            self.uavcan_response = -1
                    self.send_req = False

    async def async_activate_headlight_publisher(self):
        while 1:
            await asyncio.sleep(0.0001)
            if self.send_activate_headlight:
                await self.headlight_activate_publisher.publish(uavcan.primitive.scalar.Bit_1_0(self.activate_headlight))
                self.send_activate_headlight = False

    def close(self):
        self.headlight_uavcan_node.close()

class HeadlightRosInterface(Node):
    def __init__(self):
        super().__init__('headlight_uavcan_srv')

        self.get_logger().info("Initialising headlight_uavcan_srv node")

        self.declare_parameter('uavcan.local_node_name', 'rossrobotics.mk4.ccu.bumper')
        self.uavcan_local_node_name = self.get_parameter('uavcan.local_node_name').value

        self.declare_parameter('uavcan.bumper_node_name', 'rossrobotics.mk4.bumper')
        self.uavcan_bumper_node_name = self.get_parameter('uavcan.bumper_node_name').value

        self.declare_parameter('uavcan.publisher_port_name', 'activate_headlights')
        self.uavcan_pub_name = self.get_parameter('uavcan.publisher_port_name').value

        self.declare_parameter('uavcan.can_interface', 'can0')
        self.uavcan_can_interface = self.get_parameter('uavcan.can_interface').value

        self.declare_parameter('uavcan.mtu', 8)
        self.uavcan_mtu = self.get_parameter('uavcan.mtu').value

        self.declare_parameter('uavcan.local_node_id', 4)
        self.uavcan_local_node_id = self.get_parameter('uavcan.local_node_id').value

        self.declare_parameter('uavcan.version.major', 1)
        self.uavcan_v_major = self.get_parameter('uavcan.version.major').value

        self.declare_parameter('uavcan.version.minor', 0)
        self.uavcan_v_minor = self.get_parameter('uavcan.version.minor').value

        self.declare_parameter('uavcan.register.file_path', '/home/pi/headlight_interface.db')
        self.uavcan_register_file_path = self.get_parameter('uavcan.register.file_path').value

        self.declare_parameter('uavcan.register.publisher.key_name', 'uavcan.pub.activate_headlights.id')
        self.uavcan_register_pub_key_name = self.get_parameter('uavcan.register.publisher.key_name').value

        self.declare_parameter('uavcan.register.publisher.value', 1639)
        self.uavcan_register_pub_value = self.get_parameter('uavcan.register.publisher.value').value

        self.declare_parameter('uavcan.register.bitrate.key_name', 'uavcan.can.bitrate')
        self.uavcan_register_bitrate_key_name = self.get_parameter('uavcan.register.bitrate.key_name').value

        self.declare_parameter('uavcan.register.bitrate.value', '1000000 1000000')
        self.uavcan_register_bitrate_value = self.get_parameter('uavcan.register.bitrate.value').value

        self.declare_parameter('uavcan.execute_command_identifier', 18)
        self.uavcan_execute_command_identifier = self.get_parameter('uavcan.execute_command_identifier').value

        self.declare_parameter('ros.heartbeat_topic_name', 'uavcan_node_tracker')
        ros_heartbeat_topic_name = self.get_parameter('ros.heartbeat_topic_name').value

        self.declare_parameter('ros.service_name','rossrobotics_mk4_bumper/headlight')
        ros_service_name = self.get_parameter('ros.service_name').value

        self.ros_heartbeat_subscriber = self.create_subscription(Heartbeat, ros_heartbeat_topic_name, self.node_tracker_callback, 5)
        self.ros_headlight_service = self.create_service(HeadlightSrv, ros_service_name, self.headlight_srv_callback)

        self.id_name_dict = {}
        self.id_alive_dict = {}
        self.id_command_dict = {}

        self.headlight_uavcan_interface = HeadlightUavcanInterface.__new__(HeadlightUavcanInterface)
        self.uavcan_node_alive = False

        self.get_logger().info("headlight_uavcan_srv node initialised")

    def node_tracker_callback(self, msg):
        if self.uavcan_bumper_node_name in msg.header.frame_id:
            if msg.node_id not in self.id_name_dict:
                self.id_alive_dict[msg.node_id] = msg.alive
                self.id_name_dict[msg.node_id] = msg.header.frame_id
            else:
                self.id_alive_dict[msg.node_id] = msg.alive

    def headlight_srv_callback(self, request, response):
        headlight_to_call = request.headlight
        mode_byte = request.mode.to_bytes(1,byteorder='big')
        ints_byte_array = bytearray(struct.pack("f", request.intensity))

        headlight_to_call_ids = []
        response_message = ""

        if len(self.id_name_dict) != len(headlight_to_call):
            response_message = response_message + 'The requested number of headlights have not been found on the CAN bus. Requested: ' \
                + str(len(headlight_to_call)) + ', Found: ' \
                + str(len(self.id_name_dict)) + ' on the CAN bus. '

        for key in self.id_name_dict:
            for headlight in headlight_to_call:
                if headlight in self.id_name_dict[key]:
                    headlight_to_call_ids.append(key)

        self.headlight_uavcan_interface.activate_headlight = False
        self.headlight_uavcan_interface.send_activate_headlight = True

        for id in headlight_to_call_ids:
            if self.id_alive_dict[id]:
                self.headlight_uavcan_interface.id_command_dict[id] = mode_byte + ints_byte_array
                self.headlight_uavcan_interface.send_req = True

                while self.headlight_uavcan_interface.uavcan_response == None:
                    pass

                if self.headlight_uavcan_interface.uavcan_response == self.headlight_uavcan_interface.RESPONSE_SUCCESS:
                    response.success = True
                    response_message = response_message + 'Uavcan node: ' + self.id_name_dict[id] + ', ID: ' \
                        + str(id) + \
                        '. Successfully set. '
                elif self.headlight_uavcan_interface.uavcan_response == self.headlight_uavcan_interface.RESPONSE_FAILURE:
                    response.success = False
                    response_message = response_message + 'Uavcan node: ' + self.id_name_dict[id] + ', ID: ' \
                        + str(id) + \
                        '. Headlight is disconnected or malfunctioning. '
                elif self.headlight_uavcan_interface.uavcan_response == self.headlight_uavcan_interface.RESPONSE_BAD_PARAMETER:
                    response.success = False
                    response_message = response_message + 'Uavcan node: ' + self.id_name_dict[id] + ', ID: ' \
                        + str(id) + \
                        '. Not successful. Mode selected not available. '
                elif self.headlight_uavcan_interface.uavcan_response == self.headlight_uavcan_interface.RESPONSE_NO_RESPONSE:
                    response.success = False
                    response_message = response_message + 'Uavcan node: ' + self.id_name_dict[id] + ', ID: ' \
                        + str(id) + \
                        '. Not sucessful. No response. '

                self.headlight_uavcan_interface.uavcan_response = None
                self.headlight_uavcan_interface.id_command_dict[id] = None
            else:
                response.success = False
                response_message = response_message + 'Uavcan node: ' + self.id_name_dict[id] + ', ID: ' \
                        + str(id) + \
                        '. Not alive. '

        self.headlight_uavcan_interface.activate_headlight = True
        self.headlight_uavcan_interface.send_activate_headlight = True

        if len(self.id_name_dict) != len(headlight_to_call):
            response.success = False
        response.message = response_message

        return response

class Headlight():
    def __init__(self):
        self.ros_headlight_interface = HeadlightRosInterface()
        self.uavcan_headlight_interface = HeadlightUavcanInterface(\
            self.ros_headlight_interface.uavcan_can_interface,\
            self.ros_headlight_interface.uavcan_mtu, \
            self.ros_headlight_interface.uavcan_local_node_id, \
            self.ros_headlight_interface.uavcan_local_node_name, \
            self.ros_headlight_interface.uavcan_v_major, \
            self.ros_headlight_interface.uavcan_v_minor, \
            self.ros_headlight_interface.uavcan_register_file_path,\
            self.ros_headlight_interface.uavcan_register_bitrate_key_name,\
            self.ros_headlight_interface.uavcan_register_bitrate_value, \
            self.ros_headlight_interface.uavcan_execute_command_identifier, \
            self.ros_headlight_interface.uavcan_pub_name, \
            self.ros_headlight_interface.uavcan_register_pub_key_name, \
            self.ros_headlight_interface.uavcan_register_pub_value)

        self.ros_headlight_interface.headlight_uavcan_interface = self.uavcan_headlight_interface
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.ros_headlight_interface)

    async def loop(self):
        while 1:
            await asyncio.sleep(0.0001)
            for key in self.ros_headlight_interface.id_name_dict:
                if key not in self.uavcan_headlight_interface.id_name_dict:
                    self.uavcan_headlight_interface.id_name_dict[key] = self.ros_headlight_interface.id_name_dict[key]
                    self.uavcan_headlight_interface.id_headlightclient_dict[key] = None
                    self.uavcan_headlight_interface.id_command_dict[key] = None

            self.executor.spin_once(timeout_sec = 0.0)

async def main():
    headlight = Headlight()

    try:
        task1 = asyncio.create_task(headlight.uavcan_headlight_interface.async_execute_command())
        task2 = asyncio.create_task(headlight.loop())
        task3 = asyncio.create_task(headlight.uavcan_headlight_interface.async_activate_headlight_publisher())
        await task1
        await task2
        await task3
    finally:
        headlight.uavcan_headlight_interface.close()
        pass

if __name__ == '__main__':
    rclpy.init()
    asyncio.run(main())
    rclpy.shutdown()