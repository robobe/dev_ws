import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

class TestParams(Node):

    def __init__(self):
        super().__init__('simple_params')

        self.declare_parameter('my_str')
        self.declare_parameter('my_int', value=10)
        self.declare_parameter('my_double_array')
        self.add_on_set_parameters_callback(self.parameter_callback)
        timer_period = 2
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def parameter_callback(self, params):
        print(type(params))
        for param in params:
            self.get_logger().info(param.name)
            self.get_logger().info(str(param.value))
            self.get_logger().info(str(param.type_))
        return SetParametersResult(successful=True)

    def timer_callback(self):
        my_param = self.get_parameter('my_str').get_parameter_value().string_value
        my_param_int = self.get_parameter("my_int").get_parameter_value().integer_value
        my_param_array = self.get_parameter("my_double_array").get_parameter_value().double_array_value
        self.get_logger().info(f"Hello {my_param}! with int data: {my_param_int}")
        self.get_logger().info(str(my_param_array))

def main(args=None):
    rclpy.init(args=args)
    node = TestParams()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()