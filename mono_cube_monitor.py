import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import numpy as np
from collections import deque

TOPIC = "/ai_cube/position_camera"

class Monitor(Node):
    def __init__(self):
        super().__init__("mono_cube_monitor")
        self.buf = deque(maxlen=30)
        self.create_subscription(PointStamped, TOPIC, self.cb, 20)

    def cb(self, msg):
        p = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=float)
        self.buf.append(p)
        if len(self.buf) >= 10:
            a = np.asarray(self.buf)
            med = np.median(a, axis=0)
            spread = np.max(a, axis=0) - np.min(a, axis=0)
            self.get_logger().info(
                f"median=({med[0]:+.4f},{med[1]:+.4f},{med[2]:+.4f}) "
                f"spread=({spread[0]*1000:.1f},"
                f"{spread[1]*1000:.1f},{spread[2]*1000:.1f}) mm "
                f"n={len(a)}"
            )

def main():
    rclpy.init()
    n = Monitor()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()