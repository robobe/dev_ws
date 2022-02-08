import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

ANGULAR_SPEED = 0.3
LINEAR_SPEED = 0.6
ONE_METER = 1

class MyNode(Node):
    def __init__(self):
        super().__init__('skbot_motion_scan')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT,
            depth=1
        )

        
        self.__twist_pub = self.create_publisher(Twist, 
            "/skbot/cmd_vel",  
            qos_profile)

        self.sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.__scan_handler,
            qos_profile
        )

    def __scan_handler(self, msg: LaserScan):
        # 720 / 5 = 144
        regions = {
            "left": min(min(msg.ranges[0:143]), 10),
            "fleft": min(min(msg.ranges[144:287]), 10),
            "front": min(min(msg.ranges[288:431]), 10),
            "fright": min(min(msg.ranges[432:575]), 10),
            "right": min(min(msg.ranges[576:713]), 10),
        }
        self.__take_action(regions)

    def __take_action(self, regions):
        msg = Twist()
        linear_x = 0.0
        angular_z = 0.0

        state_description = ''

        if regions['front'] > ONE_METER and regions['fleft'] > ONE_METER and regions['fright'] > ONE_METER:
            state_description = 'case 1 - nothing'
            linear_x = LINEAR_SPEED
            angular_z = 0.0
        elif regions['front'] < ONE_METER and regions['fleft'] > ONE_METER and regions['fright'] > ONE_METER:
            state_description = 'case 2 - front'
            linear_x = 0
            angular_z = -ANGULAR_SPEED
        elif regions['front'] > ONE_METER and regions['fleft'] > ONE_METER and regions['fright'] < ONE_METER:
            state_description = 'case 3 - fright'
            linear_x = 0
            angular_z = -ANGULAR_SPEED
        elif regions['front'] > ONE_METER and regions['fleft'] < ONE_METER and regions['fright'] > ONE_METER:
            state_description = 'case 4 - fleft'
            linear_x = 0
            angular_z = ANGULAR_SPEED
        elif regions['front'] < ONE_METER and regions['fleft'] > ONE_METER and regions['fright'] < ONE_METER:
            state_description = 'case 5 - front and fright'
            linear_x = 0
            angular_z = -ANGULAR_SPEED
        elif regions['front'] < ONE_METER and regions['fleft'] < ONE_METER and regions['fright'] > ONE_METER:
            state_description = 'case 6 - front and fleft'
            linear_x = 0
            angular_z = ANGULAR_SPEED
        elif regions['front'] < ONE_METER and regions['fleft'] < ONE_METER and regions['fright'] < ONE_METER:
            state_description = 'case 7 - front and fleft and fright'
            linear_x = 0
            angular_z = -ANGULAR_SPEED
        elif regions['front'] > ONE_METER and regions['fleft'] < ONE_METER and regions['fright'] < ONE_METER:
            state_description = 'case 8 - fleft and fright'
            linear_x = 0
            angular_z = -ANGULAR_SPEED
        else:
            state_description = 'unknown case'

        self.get_logger().info(state_description)
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.__twist_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()