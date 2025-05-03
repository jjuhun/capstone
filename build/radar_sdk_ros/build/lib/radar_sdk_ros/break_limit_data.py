import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import threading

import numpy as np
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

        # ✅ 프레임 수집 무한 루프 실행
        thread = threading.Thread(target=self.publish_frames)
        thread.daemon = True
        thread.start()

    def publish_frames(self):
        while rclpy.ok():
            frame_contents = self.device.get_next_frame()
            for frame in frame_contents:
                num_rx = np.shape(frame)[0]

                for iAnt in range(num_rx):
                    mat = frame[iAnt, :, :]
                    mat_normalized = ((mat + 1) / 2 * 4095).astype(int)
                    mat_flattened = mat_normalized.flatten()

                    msg = Int32MultiArray()
                    msg.data = mat_flattened.tolist()
                    self.publisher_.publish(msg)

                    self.get_logger().info(
                        f'Frame {self.frame_number}, Antenna {iAnt} 전송 완료 (Size={len(mat_flattened)})'
                    )
            self.frame_number += 1

def main(args=None):
    rclpy.init(args=args)
    node = RadarPublisher()
    rclpy.spin(node)  # Ctrl+C 시 자동 종료
    node.device.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
