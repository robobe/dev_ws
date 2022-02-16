from sre_parse import State
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import math
from enum import IntEnum

DESIRED_POSE = Point()
DESIRED_POSE.x = -5.0
DESIRED_POSE.y = 8.0


class EulerEnum(IntEnum):
    ROLL = 0
    PITCH = 1
    YAW = 2


class StateEnum(IntEnum):
    GO_TO_POINT = 0
    WALL_FOLLOWING = 1


def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z


QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT,
    depth=1,
)


class MyNode(Node):
    def __init__(self):
        super().__init__("skbot_bug0")
        self.__pose = None
        self.__yaw = None
        self.__regions = None
        self.__state = StateEnum.GO_TO_POINT

        self.__goto_point_srv = self.create_client(SetBool, "go_to_point_switch")
        self.__wall_follower_srv = self.create_client(SetBool, "wall_follower_switch")
        self.__odom_sub = self.create_subscription(
            Odometry, "/skbot/odom", self.__odom_handler, QOS
        )

        self.__laser_scan_sub = self.create_subscription(
            LaserScan, "/scan", self.__scan_handler, QOS
        )

        self.__timer = self.create_timer(1 / 5, self.__timer_handler)
        self.__change_state(StateEnum.WALL_FOLLOWING)
        # End init

    def __normalize_angle(self, angle):
        """
        angel = 200 degree -> 3.49 rad
        3.49 - 6.28*3.49/3.49
        """
        if math.fabs(angle) > math.pi:
            angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
        return angle

    def __timer_handler(self):
        if not self.__regions:
            return
        if self.__state == StateEnum.GO_TO_POINT:
            if self.__regions["front"] < 1:
                # close to wall switch to wall follow
                self.__change_state(StateEnum.WALL_FOLLOWING)
        elif self.__state == StateEnum.WALL_FOLLOWING:
            desired_yaw = math.atan2(
                DESIRED_POSE.y - self.__pose.y, DESIRED_POSE.x - self.__pose.x
            )
            err_yaw = self.__normalize_angle(desired_yaw - self.__yaw)

            if self.__regions["front"]:
                # if yaw_error < 30 and we farway from wall the switch
                self.__change_state(StateEnum.GO_TO_POINT)

            # if err_yaw > 0 and \
            #    math.fabs(err_yaw) > (math.pi / 4) and \
            #    math.fabs(err_yaw) < (math.pi / 2) and \
            #    self.__regions['left'] > 1.5:
            #     # if yaw error positive and 45 < yaw_errot < 90
            #     self.__change_state(StateEnum.GO_TO_POINT)

            # if err_yaw < 0 and \
            #    math.fabs(err_yaw) > (math.pi / 4) and \
            #    math.fabs(err_yaw) < (math.pi / 2) and \
            #    self.__regions['right'] > 1.5:
            #     self.__change_state(StateEnum.GO_TO_POINT)

    def __change_state(self, state):
        self.__state = state
        self.get_logger().info(f"Change state: {state.name}")
        start_req = SetBool.Request(data=True)
        stop_req = SetBool.Request(data=False)
        if self.__state == StateEnum.GO_TO_POINT:
            resp = self.__goto_point_srv.call_async(start_req)
            resp = self.__wall_follower_srv.call_async(stop_req)
        if self.__state == StateEnum.WALL_FOLLOWING:
            resp = self.__goto_point_srv.call_async(stop_req)
            resp = self.__wall_follower_srv.call_async(start_req)

    def __scan_handler(self, msg: LaserScan):
        self.__regions = {
            "right": min(min(msg.ranges[0:143]), 10),
            "fright": min(min(msg.ranges[144:287]), 10),
            "front": min(min(msg.ranges[288:431]), 10),
            "fleft": min(min(msg.ranges[432:575]), 10),
            "left": min(min(msg.ranges[576:719]), 10),
        }

    def __odom_handler(self, msg: Odometry):
        """
        ros2 topic echo /skbot/odom | grep -A 4 "position"
        ros2 topic echo /skbot/odom | grep -A 4 "orientation"
        """
        self.__pose = msg.pose.pose.position

        euler = euler_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        self.__yaw = euler[EulerEnum.YAW]


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
