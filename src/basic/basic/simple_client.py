import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class MinimalClient(Node):

    def __init__(self):
        super().__init__('minimal_client')
        self.client = self.create_client(SetBool, 'echo_service')
        self.timer = self.create_timer(1, self.timer_callback)
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available')
        self.request = SetBool.Request()

    def timer_callback(self):
        self.get_logger().info("time callback")

    def send_request(self):
        self.get_logger().info("Send Request")
        self.request.data = True
        wait = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, wait)
        if wait.result() is not None:
            self.get_logger().info("Request was {}".format(wait.result().message))   
        else:            
            self.get_logger().info("Request failed")

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClient()
    minimal_client.send_request()
    rclpy.spin(minimal_client)
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()