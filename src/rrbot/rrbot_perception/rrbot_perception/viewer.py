import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

CAMERA_TOPIC = "/camera1/image_raw"


class ImageViewer(Node):
    def __init__(self):
        super().__init__("viewer")
        self.image_sub = self.create_subscription(Image, CAMERA_TOPIC, self.callback, 10)
        self.bridge = CvBridge()

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv2.imshow("image", cv_image)
        cv2.waitKey(3)

def main(args=None):
    try:
        rclpy.init()
        image_converter = ImageViewer()
        rclpy.spin(image_converter)
        image_converter.destroy_node
        rclpy.shutdown()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()
if __name__ == '__main__':   
    main()