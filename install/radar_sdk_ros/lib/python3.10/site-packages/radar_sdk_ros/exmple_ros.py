# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Int32MultiArray

# class RadarPublisher(Node):
#     def __init__(self):
#         super().__init__('radar_publisher')
#         self.publisher_ = self.create_publisher(Int32MultiArray, 'radar_topic', 10)
#         self.buffer = []  # 데이터를 임시로 보관할 버퍼
#         self.chunk_size = 1024

#         # 레이더 센서 데이터 수집용 타이머 (예: 초당 500Hz 샘플링이라면, 0.002초 간격)
#         self.timer = self.create_timer(0.002, self.collect_data)

#     def collect_data(self):
#         new_sample = self.get_radar_sample()  # 레이더에서 하나의 샘플 얻기 (사용자가 정의)
#         self.buffer.append(new_sample)

#         if len(self.buffer) >= self.chunk_size:
#             self.publish_data()

#     def publish_data(self):
#         msg = Int32MultiArray()
#         msg.data = self.buffer.copy()
#         self.publisher_.publish(msg)
#         self.get_logger().info(f"Published {len(self.buffer)} samples.")
#         self.buffer.clear()

#     def get_radar_sample(self):
#         # 실제 레이더 데이터 샘플 얻는 로직으로 교체
#         sample = 0
#         return sample

# def main(args=None):
#     rclpy.init(args=args)
#     node = RadarPublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
