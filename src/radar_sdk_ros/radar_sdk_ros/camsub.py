import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CamSub(Node):
    def __init__(self):
        super().__init__('camsub')
        self.subscription = self.create_subscription(
            Image,
            'image_topic',
            self.listener_callback,
            10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow('Received Camera Stream', cv_image)
            cv2.waitKey(1)  # 1ms 대기해서 실시간으로 보여줌
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = CamSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
