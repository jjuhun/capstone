from datetime import datetime
# === Publisher with Timestamp for Delay Measurement ===
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import threading
import numpy as np
import time
from ifxradarsdk.fmcw import DeviceFmcw
from ifxradarsdk.fmcw.types import FmcwSimpleSequenceConfig, FmcwSequenceChirp

class RadarPublisher(Node):

    def __init__(self):
        super().__init__('radar_publisher')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'radar_publisher', 10)

        self.device = DeviceFmcw()
        self.config = FmcwSimpleSequenceConfig(
            frame_repetition_time_s=21.052e-3,
            chirp_repetition_time_s=250.15e-6,
            num_chirps=32,
            tdm_mimo=False,
            chirp=FmcwSequenceChirp(
                start_frequency_Hz=59.9e9,
                end_frequency_Hz=61.6e9,
                sample_rate_Hz=1e6,
                num_samples=128,
                rx_mask=2,
                tx_mask=1,
                tx_power_level=31,
                lp_cutoff_Hz=500000,
                hp_cutoff_Hz=80000,
                if_gain_dB=30,
            ),
        )
        sequence = self.device.create_simple_sequence(self.config)
        self.device.set_acquisition_sequence(sequence)
        self.frame_number = 0
        self.start_time_ms = int(time.time() * 1000)  # 기준 시간

        thread = threading.Thread(target=self.publish_frames)
        thread.start()

    def publish_frames(self):
        start_time = time.time()  # 시작 시간 기록

        while self.frame_number < 128:
            frame_contents = self.device.get_next_frame()
            for frame in frame_contents:
                num_rx = np.shape(frame)[0]
                for iAnt in range(num_rx):
                    mat = frame[iAnt, :, :]
                    mat_normalized = ((mat + 1) / 2 * 4095).astype(np.int32)
                    mat_flattened = mat_normalized.flatten().astype(int)  # ensure native Python int

                    timestamp = int(time.time() * 1000) - self.start_time_ms  # 상대 시간(ms)  # ms 단위
                    msg = Int32MultiArray()
                    msg.data = [int(timestamp)] + [int(x) for x in mat_flattened]
                    self.publisher_.publish(msg)

                    # self.get_logger().info(
                    #     f'Frame {self.frame_number}, Antenna {iAnt} 전송 완료 (Size={len(mat_flattened)})'
                    # )
            self.frame_number += 1

        # self.get_logger().info('✅ 모든 128프레임 전송 완료')
                # 종료 신호 보내기
        end_msg = Int32MultiArray()
        end_msg.data = [-1]  # 종료 트리거 메시지
        self.publisher_.publish(end_msg)
        print("🕒 현재 시간(ms 포함):", datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3])

        self.get_logger().info('🛑 종료 트리거 메시지 전송 완료')

        self.device.close()

def main(args=None):
    rclpy.init(args=args)
    node = RadarPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
