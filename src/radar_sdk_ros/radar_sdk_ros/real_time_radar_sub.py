import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

import numpy as np
import threading
import matplotlib.pyplot as plt
from scipy import signal
import time

# 설정
NTS = 128
NUM_CHIRPS_PER_FRAME = 32
SAMPLES_PER_FRAME = NTS * NUM_CHIRPS_PER_FRAME
NUM_FRAMES = 64
WINDOW_SIZE = NUM_FRAMES * SAMPLES_PER_FRAME
Tsweep = 250.15e-6
fc = 61.25e9  # 중심 주파수

# 전역 버퍼
frame_buffer = np.zeros(WINDOW_SIZE, dtype=np.int16)
buffer_lock = threading.Lock()
stop_event = threading.Event()

class RadarSubscriber(Node):

    def __init__(self):
        super().__init__('radar_subscriber')
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'radar_publisher',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        global frame_buffer
        data = np.array(msg.data, dtype=np.int16)

        if data.size != SAMPLES_PER_FRAME:
            self.get_logger().warn(f"잘못된 데이터 크기: {data.size}, 예상: {SAMPLES_PER_FRAME}")
            return

        with buffer_lock:
            frame_buffer = np.roll(frame_buffer, -SAMPLES_PER_FRAME)
            frame_buffer[-SAMPLES_PER_FRAME:] = data

def process_and_show():
    PRF = 1 / Tsweep
    bin_indl = 10
    bin_indu = 30
    window_length = 200
    overlap_factor = 0.95
    overlap_length = int(round(window_length * overlap_factor))
    pad_factor = 4
    fft_points = pad_factor * window_length

    plt.ion()
    fig, ax = plt.subplots()
    dummy_data = np.zeros((256, 100))
    im = ax.imshow(dummy_data, aspect='auto', cmap='jet', origin='lower')
    cbar = plt.colorbar(im)
    plt.show()

    while not stop_event.is_set():
        with buffer_lock:
            buffer_copy = frame_buffer.copy()

        try:
            data_time = buffer_copy.reshape((NTS, -1), order='F')
        except ValueError:
            continue

        tmp = np.fft.fftshift(np.fft.fft(data_time, axis=0), axes=0)
        data_range = tmp[NTS//2:, :]

        ns = data_range.shape[1]
        b, a = signal.butter(4, 0.0075, 'high')
        data_range_MTI = np.zeros((data_range.shape[0], ns), dtype=np.float32)
        for k in range(data_range.shape[0]):
            data_range_MTI[k, :] = signal.lfilter(b, a, np.real(data_range[k, :]))

        f, t, _ = signal.spectrogram(
            data_range_MTI[bin_indl, :],
            fs=PRF,
            window='hamming',
            nperseg=window_length,
            noverlap=overlap_length,
            nfft=fft_points,
            return_onesided=False,
            scaling='density'
        )
        f = np.fft.fftshift(f)
        data_spec = np.zeros((len(f), len(t)))

        for RBin in range(bin_indl, bin_indu):
            _, _, Sxx = signal.spectrogram(
                data_range_MTI[RBin, :],
                fs=PRF,
                window='hamming',
                nperseg=window_length,
                noverlap=overlap_length,
                nfft=fft_points,
                return_onesided=False,
                scaling='density'
            )
            Sxx = np.fft.fftshift(Sxx, axes=0)
            data_spec += np.abs(Sxx)

        data_spec /= np.max(np.abs(data_spec)) + 1e-10
        data_spec = 20 * np.log10(data_spec + 1e-10)
        data_spec = (data_spec + 100) * 0.3 + 100
        velocity_axis = f * 3e8 / (2 * fc)

        im.set_data(data_spec)
        im.set_extent([t[0], t[-1], velocity_axis[0], velocity_axis[-1]])
        im.set_clim(np.min(data_spec), np.max(data_spec))
        fig.canvas.draw()
        fig.canvas.flush_events()

        plt.pause(0.05)

    plt.ioff()
    plt.close()

def main(args=None):
    rclpy.init(args=args)
    radar_node = RadarSubscriber()

    # 🔄 ROS 노드를 스레드로 실행
    ros_thread = threading.Thread(target=rclpy.spin, args=(radar_node,))
    ros_thread.start()

    try:
        # 🖼️ 시각화는 메인 스레드에서 실행 (GUI는 메인에서!)
        process_and_show()
    except KeyboardInterrupt:
        stop_event.set()

    ros_thread.join()
    radar_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

