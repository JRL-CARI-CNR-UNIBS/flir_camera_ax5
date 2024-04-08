import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import time

class ColorMap(Node):
  def __init__(self):
    super().__init__("colormap_node")

    self._sub = self.create_subscription(Image, "/flir/ax5_camera_node/image_raw", self.color_image, 10)
    self._pub = self.create_publisher(Image, "/flir/ax5_camera_node/image_colormap", 10)
    self._cvb = CvBridge()

  def color_image(self, msg):
    tic = time.perf_counter()
    cvimage = self._cvb.imgmsg_to_cv2(msg)
    new_img = cv2.applyColorMap(cvimage, cv2.COLORMAP_PLASMA)
    msgimg: Image = self._cvb.cv2_to_imgmsg(new_img, 'bgr8')
    # cv2.imshow("debug", new_img)
    # cv2.waitKey(0)
    # print(msgimg.encoding)
    self._pub.publish(msgimg)
    print(time.perf_counter() - tic)

if __name__ == '__main__':
  rclpy.init()
  node = ColorMap()
  rclpy.spin(node)
