import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class USBCamPublisher(Node):
    def __init__(self):
        super().__init__('usb_cam_publisher')
        self.publisher_ = self.create_publisher(Image, 'usb_cam/image_raw', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('Could not open video device')
            rclpy.shutdown()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            self.publisher_.publish(msg)
        else:
            self.get_logger().error('Error reading from video device')

def main(args=None):
    rclpy.init(args=args)
    usb_cam_publisher = USBCamPublisher()
    rclpy.spin(usb_cam_publisher)
    usb_cam_publisher.cap.release()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
