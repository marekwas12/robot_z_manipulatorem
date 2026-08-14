import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters

class StereoProcessor(Node):
    def __init__(self):
        super().__init__('stereo_processor')
        self.bridge = CvBridge()
        
        # Parametry fizyczne
        self.baseline = 0.06  # 6 cm w metrach
        self.f_pixel = 243.0  # ogniskowa w pikselach dla 320px szerokości
        
        self.stereo = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=32, # Zwiększone, aby łapać obiekty bliżej
            blockSize=5
        )
        
        self.left_sub = message_filters.Subscriber(self, Image, '/camera_left/image_raw')
        self.right_sub = message_filters.Subscriber(self, Image, '/camera_right/image_raw')
        self.ts = message_filters.ApproximateTimeSynchronizer([self.left_sub, self.right_sub], 10, 0.5)
        self.ts.registerCallback(self.stereo_callback)
        
        self.pub = self.create_publisher(Image, '/stereo/disparity', 10)
        self.get_logger().info("Stereo Processor aktywny. Baseline: 6cm")

    def stereo_callback(self, left_msg, right_msg):
        try:
            left_gray = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding='mono8')
            right_gray = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding='mono8')

            disparity = self.stereo.compute(left_gray, right_gray).astype(np.float32) / 16.0
            
            # Oblicz odległość w centrum obrazu (Z = f * B / d)
            center_x, center_y = 160, 120
            d_center = disparity[center_y, center_x]
            
            if d_center > 0:
                distance = (self.f_pixel * self.baseline) / d_center
                self.get_logger().info(f"Odległość w centrum: {distance:.2f} m")

            disp_norm = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
            self.pub.publish(self.bridge.cv2_to_imgmsg(disp_norm, encoding='mono8'))
            
        except Exception as e:
            self.get_logger().error(f"Błąd: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = StereoProcessor()
    rclpy.spin(node)
    rclpy.shutdown()