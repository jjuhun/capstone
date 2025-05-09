import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import os
import time
from collections import deque

SAVE_DIR = "/home/juhun/cam_images"
os.makedirs(SAVE_DIR, exist_ok=True)

BUFFER_DURATION = 3  # 최근 3초간의 프레임만 유지
FPS = 10             # 예상 수신 FPS (초당 프레임 수)
MAX_BUFFER_SIZE = BUFFER_DURATION * FPS

class CamSub(Node):
    def __init__(self):
        super().__init__('camsub')
        self.bridge = CvBridge()

        # 이미지 및 제어 구독
        self.subscription = self.create_subscription(Image, 'image_topic', self.listener_callback, 10)
        self.control_sub = self.create_subscription(String, 'radar_control', self.control_callback, 10)

        # 버퍼: (timestamp, image) 튜플 저장
        self.frame_buffer = deque(maxlen=MAX_BUFFER_SIZE)
        self.saving = False

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            timestamp = time.time()
            self.frame_buffer.append((timestamp, cv_image))

            # 실시간 화면 출력
            cv2.imshow('Received Camera Stream', cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {str(e)}')

    def control_callback(self, msg):
        if msg.data.lower() == "save":
            self.saving = True
            self.get_logger().info(f"💾 저장 요청 수신: 최근 {BUFFER_DURATION}초 프레임 저장 중...")

            save_time = time.time()
            frames_to_save = [
                (ts, frame) for (ts, frame) in self.frame_buffer
                if save_time - ts <= BUFFER_DURATION
            ]

            for ts, frame in frames_to_save:
                timestamp = time.strftime('%Y%m%d_%H%M%S', time.localtime(ts))
                millis = int((ts % 1) * 1000)
                filename = os.path.join(SAVE_DIR, f"camera_{timestamp}_{millis:03d}.jpg")
                cv2.imwrite(filename, frame)
                self.get_logger().info(f"🖼️ 저장됨: {filename}")

            self.get_logger().info("✅ 저장 완료")
            self.saving = False

def main(args=None):
    rclpy.init(args=args)
    node = CamSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
