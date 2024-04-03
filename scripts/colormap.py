import cv2
from cv_bridge import CvBridge
import rclpy
from sensor_msgs.msg import Image

class ColorMap(rclpy.Node):
  def __init__(self):
    super.__init__("colormap_node")

    _sub = self.create_subscription(Image, "/image_raw", self.color_image, 10)
    _pub = self.create_publisher(Image, "/image_colormap", 10)
    _cvb = CvBridge()

  def color_image(self, msg):
    cvimage = self._cvb.imgmsg_to_cv2(msg)
    new_img = cv2.applyColorMap(cvimage, cv2.COLORMAP_PLASMA)
    msgimg = self._cvb.cv2_to_imgmsg(new_img)
    self._pub.publish(msgimg)

if __name__ == '__main__':
  rclpy.init()
  node = ColorMap()
  rclpy.spin(node)