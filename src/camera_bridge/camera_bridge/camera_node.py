import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        # Temat z końcówką /compressed jest standardem w ROS
        self.publisher = self.create_publisher(CompressedImage, '/camera/image_raw/compressed', 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)
        
        # Sprawdzenie czy kamera działa
        if not self.cap.isOpened():
            self.get_logger().error('Nie można otworzyć kamery!')
            
        self.timer = self.create_timer(0.1, self.timer_callback) # 10 Hz

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Tworzenie wiadomości CompressedImage
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = "jpeg"
            
            # Kompresja klatki do formatu JPEG (jakość 80)
            _, compressed = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            msg.data = np.array(compressed).tobytes()
            
            self.publisher.publish(msg)
        else:
            self.get_logger().warn('Nie można odczytać klatki!')

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()