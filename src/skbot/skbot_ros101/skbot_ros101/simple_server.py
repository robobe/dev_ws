import rclpy
from rclpy.node import Node
from skbot_interfaces.srv import AddTwoInts

class SimpleSRV(Node):
    def __init__(self):
        super().__init__("Simple_srv")
        self.__service = self.create_service(AddTwoInts, "simple_service", self.__srv_handler)
        self.get_logger().info("Server Started")


    def __srv_handler(self, request: AddTwoInts.Request, response: AddTwoInts.Response):
        self.get_logger().info("info msg")
        response.sum = request.a + request.b
        return response

def main(args=None):
    rclpy.init(args=args)
    node = SimpleSRV()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
