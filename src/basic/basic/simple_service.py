import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import time
class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(SetBool, 'echo_service', self.echo_callback)
        self.get_logger().info("Start service")

    def echo_callback(self, request, response: SetBool.Response):
        self.get_logger().info(str(type(request)))
        self.get_logger().info(str(type(response)))
        self.get_logger().info("Incoming request")
        response.success = True
        response.message = "success"
        time.sleep(5)
        return response


def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()