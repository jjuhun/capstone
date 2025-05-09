import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

import cv2
import numpy as np
import threading
import matplotlib.pyplot as plt
from scipy import signal
from tensorflow.keras.models import load_model
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


    # 스펙트로그램 파라미터
    PRF = 1 / Tsweep
    bin_indl = 10
    bin_indu = 30
    window_length = 200
    overlap_factor = 0.95
    overlap_length = int(round(window_length * overlap_factor))
    pad_factor = 4
    fft_points = pad_factor * window_length

    # 모델 로드
    model = load_model("/home/juhun/Capstone_ws/src/radar_sdk_ros/model/radar_model_250322.h5")
    class_names = ['run', 'sit', 'up', 'walk']  # 실제 학습 클래스 순서

    # 시각화 설정
    plt.ion()
    fig, ax = plt.subplots()
    dummy_data = np.zeros((256, 100))
    im = ax.imshow(dummy_data, aspect='auto', cmap='jet', origin='lower', vmin=100, vmax=130)
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

        # 로그 스케일 변환 및 정규화
        data_spec /= np.max(np.abs(data_spec)) + 1e-10
        data_spec = 20 * np.log10(data_spec + 1e-10)
        data_spec = (data_spec + 100) * 0.3 + 100

        velocity_axis = f * 3e8 / (2 * fc)

        fig_img = plt.figure(figsize=(1.28,1.28), dpi = 100)
        ax_img = fig_img.add_axes([0,0,1,1])
        ax_img.axis("off")
        ax_img.imshow(
            data_spec,
            extent=[t[0], t[-1], velocity_axis[0], velocity_axis[-1]],
            aspect='auto',
            cmap='jet',
            vmin=100, vmax =130
        )
        fig_img.canvas.draw()
        img_array = np.array(fig_img.canvas.renderer.buffer_rgba())[:,:,:3]
        plt.close(fig_img)
        img_input = cv2.resize(img_array, (128,128))
        img_input = img_input.astype(np.float32) / 255.0
        img_input =  np.expand_dims(img_input, axis=0)
        # # 예측용 이미지 변환
        # img_input = cv2.resize(data_spec, (128, 128))
        # print(model.input_shape)
        # img_input = np.stack([img_input]*3, axis=-1)  # (H, W) → (H, W, 3)
        # img_input = img_input.astype(np.float32) / 255.0
        # img_input = np.expand_dims(img_input, axis=0)  # 배치 차원

        # 예측 수행
        pred = model.predict(img_input, verbose=0)
        pred_label = np.argmax(pred)
        pred_prob = np.max(pred)
        # 그래프 갱신
        im.set_data(data_spec)
        im.set_extent([t[0], t[-1], velocity_axis[0], velocity_axis[-1]])
        im.set_clim(100, 130)
        im.set_data(data_spec)
        # print(f"Prediction: {class_names[pred_label]} ({pred_prob:.2f})")
        # fig.suptitle(f"Prediction: {class_names[pred_label]} ({pred_prob:.2f})")
        fig.suptitle(f"Prediction: {class_names[pred_label]}", fontsize=60)

        fig.canvas.draw()
        fig.canvas.flush_events()
        plt.pause(0.01)

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