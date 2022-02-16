import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from std_srvs.srv import SetBool
from nav_msgs.msg import Odometry
# TODO: check it
from transforms3d.euler import euler2quat, quat2euler
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from enum import IntEnum

ANGULAR_SPEED = 0.3
LINEAR_SPEED = 0.3
ONE_METER = 1

DESIRED_POSE: Point = Point(x=-5.0, y=7.0, z=0.0)
DESIRED_PRECISION = 0.3
DESIRED_YAW = math.pi / 90
WORKING_HZ = 10
# https://quaternions.online/
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

class EulerEnum(IntEnum):
    ROLL = 0
    PITCH = 1
    YAW = 2

class StateEnum(IntEnum):
    YAW = 0
    FORWARD = 1
    DONE = 2

class MyNode(Node):
    def __init__(self):
        super().__init__('skbot_motion_scan')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT,
            depth=1
        )
        self.__state = StateEnum.YAW
        self.__pose: Point = None
        self.__yaw: float = 0
        self.__active = False

        self.__change_state(self.__state)
        self.create_timer(1/WORKING_HZ, self.__state_machine)

        self.__twist_pub = self.create_publisher(Twist, 
            "/skbot/cmd_vel",  
            qos_profile)

        self.sub = self.create_subscription(
            Odometry,
            "/skbot/odom",
            self.__odom_handler,
            qos_profile
        )

        self.__service = self.create_service(SetBool, "go_to_point_switch", self.__srv_handler)
    
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
            msg.pose.pose.orientation.w
        )
        self.__yaw = euler[EulerEnum.YAW]
        

    def __srv_handler(self, req:SetBool.Request, resp:SetBool.Response):
        self.__active = req.data
        resp.success = True
        resp.message = "Done"
        return resp

    def __calc_yaw_error(self):
        desired_yaw = math.atan2(
            DESIRED_POSE.y - self.__pose.y,
            DESIRED_POSE.x - self.__pose.x
        )
        yaw_err = desired_yaw - self.__yaw
        
        return yaw_err

    def __fix_yaw(self):
        yaw_err = self.__calc_yaw_error()
        z = ANGULAR_SPEED
        
        if math.fabs(yaw_err) > DESIRED_YAW:
            if yaw_err < 0:
                z = -ANGULAR_SPEED
            
            self.__pub_twist(0, z)
        else:
            self.__change_state(StateEnum.FORWARD)

    def __go_straight(self):
        err_pos = math.sqrt(
            pow(DESIRED_POSE.y - self.__pose.y, 2) + pow(DESIRED_POSE.x - self.__pose.x, 2)
        )

        if err_pos > DESIRED_PRECISION:
            self.__pub_twist(LINEAR_SPEED, 0)
        else:
            self.__change_state(StateEnum.DONE)

        yaw_err = self.__calc_yaw_error()
        if math.fabs(yaw_err) > DESIRED_YAW:
            self.__change_state(StateEnum.YAW)
        

    def __state_machine(self):
        if not self.__active:
            return
        if self.__state == StateEnum.YAW:
            self.__fix_yaw()
        elif self.__state == StateEnum.FORWARD:
            self.__go_straight()
        elif self.__state == StateEnum.DONE:
            self.__done()
   

    def __done(self):
        self.__pub_twist(0, 0)

    def __pub_twist(self, x, z):
        twist_msg = Twist()
        twist_msg.linear.x = float(x)
        twist_msg.angular.z = float(z)
        self.__twist_pub.publish(twist_msg)

    def __change_state(self, state: StateEnum):
        self.__state = state
        self.get_logger().info(f"State changed to: {state.name}")

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()