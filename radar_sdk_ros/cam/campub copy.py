import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import os
import time

# ✅ 저장 경로 설정
SAVE_DIR = "/home/juhun/cam_images"
os.makedirs(SAVE_DIR, exist_ok=True)

class CamPub(Node):
    def __init__(self):
        super().__init__('campub')
        self.publisher_ = self.create_publisher(Image, 'image_topic', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.2, self.timer_callback)  # 5 FPS

        # radar_control 토픽 구독
        self.control_sub = self.create_subscription(
            String,
            'radar_control',
            self.control_callback,
            10
        )

        # 카메라 설정
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # 저장 관련 상태
        self.saving = False
        self.save_duration = 3  # 저장 시간 (초)
        self.save_start_time = 0

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('카메라 프레임 수신 실패')
            return

        resized_frame = cv2.resize(frame, (32, 32))

        # 이미지 퍼블리시
        msg = self.bridge.cv2_to_imgmsg(resized_frame, encoding='bgr8')
        self.publisher_.publish(msg)

        # 저장 요청 시 이미지 저장
        if self.saving:
            now = time.time()
            if now - self.save_start_time <= self.save_duration:
                timestamp = time.strftime('%Y%m%d_%H%M%S')
                millis = int((now - self.save_start_time) * 1000)
                filename = os.path.join(SAVE_DIR, f"camera_{timestamp}_{millis}ms.jpg")
                cv2.imwrite(filename, resized_frame)
                self.get_logger().info(f"💾 저장됨: {filename}")
            else:
                self.get_logger().info("🛑 저장 종료")
                self.saving = False

    def control_callback(self, msg):
        if msg.data.lower() == "save":
            self.saving = True
            self.save_start_time = time.time()
            self.get_logger().info(f"⏱️ 저장 시작: {self.save_duration}초 동안 이미지 저장")

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
