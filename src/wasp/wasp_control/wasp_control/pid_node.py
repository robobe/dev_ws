from rclpy.node import Node
from .pid import PID

class PIDNode(Node):
    def __init__(self, name):
        super().__init__(name)