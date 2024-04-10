import sys
import os

current_dir = os.path.dirname(os.path.realpath(__file__))
print(f"Appending to sys.path: {current_dir}")
sys.path.append(current_dir)
try:
    import darknet
    print("Successfully imported darknet.")
except ModuleNotFoundError as e:
    print("Failed to import darknet after adjusting sys.path")
    print(f"Error: {e}")
    print("Current sys.path:")
    for path in sys.path:
        print(path)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import String

import ctypes




class YOLONode(Node):
    def __init__(self):
        super().__init__('yolo_detection')
        self.subscription = self.create_subscription(
            Image,
            'usb_cam/image_raw',
            self.listener_callback,
            10)
        # self.publisher_ = self.create_publisher(Image, 'yolo_detection/image_result', 10)
        self.detection_publisher = self.create_publisher(String, 'yolo_detection/detections', 10)
        self.bridge = CvBridge()
        
        # YOLO initialization
        self.data_path = '/home/weng/work/yolo_ws/src/yolov4/yolov4/node/cfg/AutoRace.data'
        self.cfg_path = '/home/weng/work/yolo_ws/src/yolov4/yolov4/node/cfg/yolov4-tiny.cfg'
        self.weights_path = '/home/weng/work/yolo_ws/src/yolov4/yolov4/node/cfg/yolov4-tiny_best_old.weights'
        # yolo_ws/src/yolov4/yolov4/node/cfg/yolov4-tiny_best_old.weights
        self.network, self.class_names, self.class_colors = darknet.load_network(
            self.cfg_path,
            self.data_path,
            self.weights_path,
            batch_size=1
        )
        self.darknet_width = darknet.network_width(self.network)
        self.darknet_height = darknet.network_height(self.network)

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        width = darknet.network_width(self.network)
        height = darknet.network_height(self.network)
        darknet_image = darknet.make_image(width, height, 3)

        
        image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        image_resized = cv2.resize(image_rgb, (width, height),
                                interpolation=cv2.INTER_LINEAR)

        darknet.copy_image_from_bytes(darknet_image, image_resized.tobytes())
        detections = darknet.detect_image(self.network, self.class_names, darknet_image, thresh=0.5)
        darknet.free_image(darknet_image)
        image = darknet.draw_boxes(detections, image_resized, self.class_colors)
        
        
        # # Convert back to ROS Image message and publish
        # image_message = self.bridge.cv2_to_imgmsg(image_with_detections_resized, encoding="bgr8")
        # self.publisher_.publish(image_message)
        if detections:
            detection_ids = ','.join([str(self.class_names.index(det[0])) for det in detections])
        else:
            detection_ids = '999'  # Special value indicating no detections

        # Publish detection results
        self.detection_publisher.publish(String(data=detection_ids))

        # Display the image with detections using OpenCV
        cv2.imshow('YOLO Detection', image)
        cv2.waitKey(1)

    def array_to_image(self, arr):
        # Ensure array is in float32 format
        arr = arr.transpose(2, 0, 1)  # Change BGR (OpenCV) to RGB (Darknet)
        arr = np.ascontiguousarray(arr, dtype=np.float32)  # Ensure contiguous array
        c, h, w = arr.shape
        arr /= 255.0  # Normalize to 0.0 - 1.0 as expected by Darknet

        # Create IMAGE structure
        image = darknet.IMAGE(w, h, c)
        data = arr.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
        ctypes.memmove(image.data, data, arr.nbytes)

        return image


def main(args=None):
    rclpy.init(args=args)
    yolo_node = YOLONode()
    rclpy.spin(yolo_node)
    yolo_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
