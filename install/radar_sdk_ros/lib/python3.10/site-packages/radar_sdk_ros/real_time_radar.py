import numpy as np
import threading
import time
import matplotlib.pyplot as plt
from scipy import signal
from ifxradarsdk.fmcw import DeviceFmcw
from ifxradarsdk.fmcw.types import FmcwSimpleSequenceConfig, FmcwSequenceChirp

# 설정
NTS = 128
NUM_CHIRPS_PER_FRAME = 32
SAMPLES_PER_FRAME = NTS * NUM_CHIRPS_PER_FRAME
NUM_FRAMES = 64
WINDOW_SIZE = NUM_FRAMES * SAMPLES_PER_FRAME
Tsweep = 250.15e-6
fc = 61.25e9  # 중심 주파수

frame_buffer = np.zeros(WINDOW_SIZE, dtype=np.int16)
buffer_lock = threading.Lock()
stop_event = threading.Event()

def odd_number(n):
    return n if n % 2 == 1 else n + 1

# 레이더 데이터 수집
def collect_radar_data():
    config = FmcwSimpleSequenceConfig(
        frame_repetition_time_s=21.052e-3,
        chirp_repetition_time_s=250.15e-6,
        num_chirps=NUM_CHIRPS_PER_FRAME,
        tdm_mimo=False,
        chirp=FmcwSequenceChirp(
            start_frequency_Hz=59.9e9,
            end_frequency_Hz=61.6e9,
            sample_rate_Hz=1e6,
            num_samples=NTS,
            rx_mask=2,
            tx_mask=1,
            tx_power_level=31,
            lp_cutoff_Hz=500000,
            hp_cutoff_Hz=80000,
            if_gain_dB=30,
        ),
    )

    global frame_buffer

    with DeviceFmcw() as device:
        sequence = device.create_simple_sequence(config)
        device.set_acquisition_sequence(sequence)

        try:
            while not stop_event.is_set():
                frame_contents = device.get_next_frame()

                for frame in frame_contents:
                    num_rx = frame.shape[0]
                    for iAnt in range(num_rx):
                        mat = frame[iAnt, :, :]
                        mat_normalized = ((mat + 1) / 2 * 4095).astype(np.int16)
                        mat_flattened = mat_normalized.flatten(order='F')

                        with buffer_lock:
                            frame_buffer = np.roll(frame_buffer, -len(mat_flattened))
                            frame_buffer[-len(mat_flattened):] = mat_flattened

                time.sleep(0.01)

        except Exception as e:
            print(f"[Radar Error]: {e}")
            stop_event.set()

# 신호처리 및 실시간 표시
def process_and_show():
    PRF = 1 / Tsweep
    bin_indl = 10
    bin_indu = 30
    window_length = 200
    overlap_factor = 0.95
    overlap_length = int(round(window_length * overlap_factor))
    pad_factor = 4
    fft_points = pad_factor * window_length

    global frame_buffer

    plt.ion()
    fig, ax = plt.subplots()
    dummy_data = np.zeros((256, 100))
    im = ax.imshow(dummy_data, aspect='auto', cmap='jet', origin='lower')
    cbar = plt.colorbar(im)
    plt.show()

    while not stop_event.is_set():
        with buffer_lock:
            buffer_copy = frame_buffer.copy()

        start_time = time.time()

        data_time = buffer_copy.reshape((NTS, -1), order='F')
        tmp = np.fft.fftshift(np.fft.fft(data_time, axis=0), axes=0)
        data_range = tmp[NTS//2:, :]

        ns = data_range.shape[1]
        b, a = signal.butter(4, 0.0075, 'high')
        data_range_MTI = np.zeros((data_range.shape[0], ns), dtype=np.float32)
        for k in range(data_range.shape[0]):
            data_range_MTI[k, :] = signal.lfilter(b, a, np.real(data_range[k, :]))

        # 스펙트로그램
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

        # 기존처럼 정규화 및 스케일링
        data_spec /= np.max(np.abs(data_spec))
        data_spec = 20 * np.log10(data_spec + 1e-10)
        data_spec = (data_spec + 100) * 0.3 + 100

        # velocity 변환 적용!!
        velocity_axis = f * 3e8 / (2 * fc)

        # 업데이트
        im.set_data(data_spec)
        im.set_extent([t[0], t[-1], velocity_axis[0], velocity_axis[-1]])
        im.set_clim(np.min(data_spec), np.max(data_spec))
        fig.canvas.draw()
        fig.canvas.flush_events()

        end_time = time.time()
        print(f"[신호처리 소요 시간]: {end_time - start_time:.3f}초")

        plt.pause(0.05)

    plt.ioff()
    plt.close()

# 메인
if __name__ == "__main__":
    radar_thread = threading.Thread(target=collect_radar_data)
    radar_thread.start()

    try:
        process_and_show()
    except KeyboardInterrupt:
        stop_event.set()

    radar_thread.join()
    print("[Radar] 정상 종료!")