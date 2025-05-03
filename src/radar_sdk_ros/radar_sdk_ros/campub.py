import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CamPub(Node):
    def __init__(self):
        super().__init__('campub')
        self.publisher_ = self.create_publisher(Image, 'image_topic', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.033, self.timer_callback)  # 약 30FPS

        # USB 캠 연결
        self.cap = cv2.VideoCapture(0)

        # 🔽 해상도 설정 (예: 640x480)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            
    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('Failed to read frame from camera.')
            return

        # 해상도 강제 축소 (예: 32x32)
        resized_frame = cv2.resize(frame, (32, 32))

        msg = self.bridge.cv2_to_imgmsg(resized_frame, encoding='bgr8')
        self.publisher_.publish(msg)

        
    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CamPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
