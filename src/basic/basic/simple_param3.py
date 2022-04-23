import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import IntegerRange
from rcl_interfaces.msg import FloatingPointRange

class TestParams(Node):

    def __init__(self):
        super().__init__('simple_params3')

        my_int_descriptor = ParameterDescriptor(
            description = "this is int",
            type = ParameterType.PARAMETER_INTEGER,
            integer_range = [IntegerRange(from_value=-10, to_value=10)])

        my_float_descriptor = ParameterDescriptor(
            description = "this is float",
            type = ParameterType.PARAMETER_DOUBLE,
            floating_point_range = [FloatingPointRange(from_value=-10.0, to_value=10.0, step=0.1)]
        )

        my_float_array_descriptor = ParameterDescriptor(
            description = "this is float array",
            type = ParameterType.PARAMETER_DOUBLE_ARRAY
        )

        my_string_descriptor = ParameterDescriptor(
            name = "format",
            description="string with constrins",
            type = ParameterType.PARAMETER_STRING,
            read_only = False,
            additional_constraints = "Supported values: [jpeg, png]"
        )
        
        self.declare_parameter('my_float', value=0.0, descriptor=my_float_descriptor)
        self.declare_parameter('my_str', value="png", descriptor=my_string_descriptor)
        self.declare_parameter('my_int', value=10, descriptor=my_int_descriptor)
        self.declare_parameter('my_double_array', value=[1.0, 2.0, 3.0], descriptor=my_float_array_descriptor)
        self.add_on_set_parameters_callback(self.parameter_callback)
        timer_period = 2
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def parameter_callback(self, params):
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