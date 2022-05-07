#!/usr/bin/env python3
import sys
compiled_dsdl_dir = "/home/pi/.yakut"

sys.path.insert(0,str(compiled_dsdl_dir))
try:
    import pyuavcan.application
except:
    print("pyuavcan application not found")

import uavcan.node  # noqa

import rclpy
from rclpy.node import Node
from mk4_msgs.msg import Heartbeat

import getmac
import asyncio
import typing
from dataclasses import dataclass

import pyuavcan
from pyuavcan.transport.can.media.socketcan import SocketCANMedia
from pyuavcan.transport.can import CANTransport
from pyuavcan.presentation import Presentation
from pyuavcan.application._node_factory import SimpleNode
from pyuavcan.application._registry_factory import make_registry
from pyuavcan.application.node_tracker import NodeTracker, Entry

class UavcanNodeTracker(Node):
    HEALTH_NOMINAL          = 0
    HEALTH_ADVISORY         = 1
    HEALTH_CAUTION          = 2
    HEALTH_WARNING          = 3

    MODE_OPERATIONAL        = 0
    MODE_INITIALISATION     = 1
    MODE_MAINTENANCE        = 2
    MODE_SOFTWARE_UPDATE    = 3

    @dataclass
    class HeartbeatInfo():
        uptime: int = 0
        mode: int = -1
        health: int = -1
        vssc: int = -1

    def __init__(self):
        super().__init__('uavcan_heartbeat_to_ros')
        self.get_logger().info('uavcan_heartbeat_to_ros node initialising')

        ## Get parameters if on the server
        self.declare_parameter('uavcan.node_name', 'rossrobotics.mk4.nodetracker')
        uavcan_node_name = self.get_parameter('uavcan.node_name').value

        self.declare_parameter('uavcan.sub_name', 'node_tracker')
        uavcan_sub_name = self.get_parameter('uavcan.sub_name').value

        self.declare_parameter('uavcan.can_interface', 'can0')
        uavcan_can_interface = self.get_parameter('uavcan.can_interface').value

        self.declare_parameter('uavcan.local_node_id', 6)
        uavcan_local_node_id = self.get_parameter('uavcan.local_node_id').value

        self.declare_parameter('uavcan.mtu', 8)
        uavcan_mtu = self.get_parameter('uavcan.mtu').value

        self.declare_parameter('uavcan.version.major', 1)
        uavcan_v_major = self.get_parameter('uavcan.version.major').value

        self.declare_parameter('uavcan.version.minor', 0)
        uavcan_v_minor = self.get_parameter('uavcan.version.minor').value

        self.declare_parameter('uavcan.timer_freq', 50)
        uavcan_timer_period = 1.0/ self.get_parameter('uavcan.timer_freq').value

        self.declare_parameter('ros.pub_freq', 2)
        ros_pub_period = 1.0/ self.get_parameter('ros.pub_freq').value

        self.declare_parameter('ros.topic_name', 'uavcan_node_tracker')
        ros_topic_name = self.get_parameter('ros.topic_name').value

        ## Create empty dictionaries
        self.id_name_dict = {}
        self.id_uniqueid_dict = {}
        self.id_alive_dict = {}
        self.id_hbinfo_dict = {}

        mac_addr = getmac.get_mac_address().split(':')
        mac_addr_int  = []
        for x in range(len(mac_addr)):
            mac_addr_int.append(int(mac_addr[x],16)) ##converts string to int

        unique_id_pad = [0] * (16-len(mac_addr_int)) ## 16 is used here as the unique id is at uint8[16] array
        ## According to their documentation the unique id should be globally unique to each node, as we will have
        ## multiple nodes on the ccu, the local node id is added to the unique id for the node. This is added in
        ## with a byte gap from the mac address so if/when debugging it is easy to decipher the two
        unique_id_pad[16-len(mac_addr_int)-2] = uavcan_local_node_id

        unique_id = unique_id_pad + mac_addr_int

        ## Setup uavcan node
        uavcan_node_info = uavcan.node.GetInfo_1.Response(
            software_version=uavcan.node.Version_1(major=uavcan_v_major, minor=uavcan_v_minor),
            name=uavcan_node_name,unique_id=unique_id)

        registry = make_registry()
        media = SocketCANMedia(uavcan_can_interface, mtu=uavcan_mtu)
        transport = CANTransport(media, local_node_id=uavcan_local_node_id)
        presentation = Presentation(transport)
        self.tracker_uavcan_node = SimpleNode(presentation,uavcan_node_info, registry)

        self.hb_sub = self.tracker_uavcan_node.make_subscriber(uavcan.node.Heartbeat_1_0, uavcan_sub_name)
        self.hb_sub.receive_in_background(self.hb_rec)
        self.tracker_uavcan_node.start()

        # Setup uavcan node tracker
        self.node_tracker = NodeTracker(self.tracker_uavcan_node)
        self.node_tracker.add_update_handler(self.uavcan_node_update_handler)

        self.uavcan_timer = self.create_timer(uavcan_timer_period, self.uavcan_timer_callback)
        self.hb_timer = self.create_timer(ros_pub_period, self.hb_to_ros_timer)

        # Create ROS publisher
        self.heartbeat_publisher = self.create_publisher(Heartbeat, ros_topic_name, 1)

        self.get_logger().info('uavcan_heartbeat_to_ros node initialised')

    def hb_to_ros_timer(self):
        for node_id in self.id_name_dict:
            if self.id_alive_dict[node_id] and self.id_name_dict[node_id] != None:
                self.id_hbinfo_dict[node_id].uptime = self.node_tracker.registry[node_id].heartbeat.uptime
                self.id_hbinfo_dict[node_id].mode = self.node_tracker.registry[node_id].heartbeat.mode.value
                self.id_hbinfo_dict[node_id].health = self.node_tracker.registry[node_id].heartbeat.health.value
                self.id_hbinfo_dict[node_id].vssc = self.node_tracker.registry[node_id].heartbeat.vendor_specific_status_code
                if self.id_hbinfo_dict[node_id].mode == self.MODE_OPERATIONAL:
                    if self.id_hbinfo_dict[node_id].health == self.HEALTH_ADVISORY:
                        msg = 'Uavcan node: ' + self.id_name_dict[node_id] + ', ID: ' \
                            + str(node_id) + ' has a minor failure dectected! This is not affecting functionality.'
                        self.get_logger().warn(msg, throttle_duration_sec=2)
                    elif self.id_hbinfo_dict[node_id].health == self.HEALTH_CAUTION:
                        msg = 'Uavcan node: ' + self.id_name_dict[node_id] + ', ID: ' \
                            + str(node_id) + ' has encountered a major failure! It has degradated performance.'
                        self.get_logger().error(msg, throttle_duration_sec=2)
                    elif self.id_hbinfo_dict[node_id].health == self.HEALTH_WARNING:
                        msg = 'Uavcan node: ' + self.id_name_dict[node_id] + ', ID: ' \
                            + str(node_id) + ' has suffered a fatal malfunction! It is unable to perform its intended functionality.'
                        self.get_logger().error(msg, throttle_duration_sec=2)
                else:
                    msg = 'Uavcan node: ' + self.id_name_dict[node_id] + ', ID: ' \
                            + str(node_id) + ' waiting for operational mode.'
                    self.get_logger().info(msg, throttle_duration_sec=5)

            if self.id_name_dict[node_id] != None:
                hb_msg = Heartbeat()
                hb_msg.header.stamp = self.get_clock().now().to_msg()
                hb_msg.header.frame_id = self.id_name_dict[node_id]
                hb_msg.alive = self.id_alive_dict[node_id]
                hb_msg.uptime = self.id_hbinfo_dict[node_id].uptime
                hb_msg.health = self.id_hbinfo_dict[node_id].health
                hb_msg.mode = self.id_hbinfo_dict[node_id].mode
                hb_msg.vssc = self.id_hbinfo_dict[node_id].vssc
                hb_msg.node_id = node_id
                self.heartbeat_publisher.publish(hb_msg)

    def uavcan_timer_callback(self):
        _loop = asyncio.get_event_loop()
        _loop.run_until_complete(self.run())

    def uavcan_node_update_handler(self, node_id: int, old: typing.Optional[Entry], new: typing.Optional[Entry]):
        if node_id not in self.id_name_dict and old == None and new != None:
            self.get_logger().info('New uavcan node has been found. ID: %d' % node_id)
            self.id_name_dict[node_id] = None
            self.id_uniqueid_dict[node_id] = None
            self.id_alive_dict[node_id] = None
            self.id_hbinfo_dict[node_id] = self.HeartbeatInfo()
        elif node_id in self.id_name_dict and new == None:
            msg = 'Uavcan node: ' + self.id_name_dict[node_id] + ', ID: ' \
                + str(node_id) + ' has gone offline.'
            self.get_logger().warn(msg)
            self.id_alive_dict[node_id] = False
        elif node_id in self.id_name_dict and old != None and new != None:
            if new.info == None:
                msg = 'Uavcan node: ' + self.id_name_dict[node_id] + ', ID: ' \
                    + str(node_id) +  ' has restarted.'
                self.get_logger().info(msg)
                self.id_alive_dict[node_id] = False
            else:
                self.id_name_dict[node_id] = new.info.name.tobytes().decode()
                self.id_uniqueid_dict[node_id] = new.info.unique_id
                self.id_alive_dict[node_id] = True
                msg = 'Uavcan node: ' + self.id_name_dict[node_id] + ', ID: ' \
                    + str(node_id) + ' has respnded to the get info request.'
                self.get_logger().info(msg)

    async def hb_rec(self, msg, meta: pyuavcan.transport.TransferFrom):
        pass

    async def run(self):
        self.hb_sub.receive_in_background(self.hb_rec)

def main(args=None):
    rclpy.init(args=args)
    uavcan_node_tracker = UavcanNodeTracker()
    rclpy.spin(uavcan_node_tracker)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)

    uavcan_node_tracker.close() # closes the uavcan node
    uavcan_node_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()