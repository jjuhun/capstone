import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, String

import numpy as np
import threading
import matplotlib.pyplot as plt
from scipy import signal
import time
import os

# 저장 폴더 설정
SAVE_DIR = '/home/juhun/radar_saves'
os.makedirs(SAVE_DIR, exist_ok=True)

# 설정
NTS = 128
NUM_CHIRPS_PER_FRAME = 32
SAMPLES_PER_FRAME = NTS * NUM_CHIRPS_PER_FRAME
NUM_FRAMES = 64
WINDOW_SIZE = NUM_FRAMES * SAMPLES_PER_FRAME
Tsweep = 250.15e-6
fc = 61.25e9  # 중심 주파수

# 전역 변수
frame_buffer = np.zeros(WINDOW_SIZE, dtype=np.int16)
buffer_lock = threading.Lock()
stop_event = threading.Event()
save_requested = threading.Event()
save_delay_seconds = 3  # 저장 전 대기 시간
save_start_time = None
saving_active = False
class RadarSubscriber(Node):

    def __init__(self):
        super().__init__('radar_subscriber')
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'radar_publisher',
            self.listener_callback,
            10
        )
        self.control_sub = self.create_subscription(
            String,
            'radar_control',
            self.control_callback,
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

    def control_callback(self, msg):
        if msg.data.lower() == 'save':
            self.get_logger().info("💾 저장 요청 수신됨! 3초 후 이미지 저장.")
            save_requested.set()

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

        # 저장 요청 처리
        if save_requested.is_set():
            save_requested.clear()
            save_start_time = time.time()
            saving_active = True
            
        if saving_active and (time.time() - save_start_time < save_duration_seconds):
            timestamp = time.strftime('%Y%m%d_%H%M%S')
            millisec = int((time.time() % 1) * 1000)
            filename = f"spectrogram_{timestamp}_{millisec:03d}.png"
            filepath = os.path.join(SAVE_DIR, filename)
            fig.savefig(filepath)
            print(f"📸 저장 중: {filepath}")
        
        elif saving_active:
            print("✅ 저장 완료")
            saving_active = False

    plt.ioff()
    plt.close()

def main(args=None):
    rclpy.init(args=args)
    radar_node = RadarSubscriber()

    # ROS 노드 스레드 실행
    ros_thread = threading.Thread(target=rclpy.spin, args=(radar_node,))
    ros_thread.start()

    try:
        process_and_show()
    except KeyboardInterrupt:
        stop_event.set()

    ros_thread.join()
    radar_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
