import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class RadarSubscriber(Node):

    def __init__(self):
        super().__init__('radar_subscriber')
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'radar_publisher',
            self.listener_callback,
            10  # QoS depth
        )

    def listener_callback(self, msg):
        data_length = len(msg.data)
        sample_data = msg.data[:10]  # 앞에서 10개만 미리보기
        self.get_logger().info(f'수신한 데이터 길이: {data_length}, 샘플: {sample_data}')

def main(args=None):
    rclpy.init(args=args)
    node = RadarSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
