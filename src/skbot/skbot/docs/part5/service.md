---
title: service
tags:
    - service
    - one on one
    - ros2
---

```python title="simple_service" linenums="1" hl_lines="3"
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
``` 

!!! tip "pylance resolve"
    Add `"python.analysis.extraPaths": []` with project path to user `settings.json`
    for example:
     ```json
     python.analysis.extraPaths": [
         "install/skbot_interfaces/lib/python3.8/site-packages/"
     ]
     ```
     

## CLI
Calling service from command line

```bash linenums="1" hl_lines="6 8"
# ros2 service call <service> <type> <data>
ros2 service call /simple_service skbot_interfaces/srv/AddTwoInts "{a: 1,b: 2}"

# Result
waiting for service to become available...
requester: making request: skbot_interfaces.srv.AddTwoInts_Request(a=1, b=2)

response:
skbot_interfaces.srv.AddTwoInts_Response(sum=3)
```

