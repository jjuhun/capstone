import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
import time

start_time = time.time()  # 시간 측정 시작

# Helper functions
def odd_number(n):
    return n if n % 2 == 1 else n + 1

def read_real_data(filename):
    rx1 = np.loadtxt(filename)
    data = rx1
    fc = 61.25e9
    Tsweep = 250.15e-6
    NTS = 128  # 정수로 명확히 설정
    Bw = 0.7e9
    header = np.array([fc, Tsweep, Bw])  # NTS는 따로 반환
    return header, int(NTS), data

def process_radar_data(filename):
    header, NTS, data = read_real_data(filename)
    fc, Tsweep, Bw = header

    # Zero-padding
    remainder = len(data) % NTS
    if remainder != 0:
        pad_length = int(NTS - remainder)
        print(f"⚠️ 데이터 길이가 {len(data)}로 NTS({NTS})에 맞지 않아 {pad_length}개를 0으로 패딩합니다.")
        data = np.pad(data, (0, pad_length), mode='constant')

    nc = int(len(data) // NTS)
    data_time = data.reshape((NTS, nc), order='F')

    win = np.ones((NTS, data_time.shape[1]))
    tmp = np.fft.fftshift(np.fft.fft(data_time * win, axis=0), axes=0)
    data_range = tmp[NTS // 2:NTS, :]
    ns = odd_number(data_range.shape[1]) - 1
    data_range_MTI = np.zeros((data_range.shape[0], ns))

    b, a = signal.butter(4, 0.0075, 'high')
    for k in range(data_range.shape[0]):
        data_range_MTI[k, :] = signal.lfilter(b, a, np.real(data_range[k, :ns]))

    data_range_MTI = data_range_MTI[1:, :]

    bin_indl, bin_indu = 10, 30
    PRF = 1 / Tsweep

    window_length = 200
    max_length = data_range_MTI.shape[1]
    if window_length > max_length:
        print(f"⚠️ window_length({window_length})이 데이터 길이({max_length})보다 커서 줄입니다.")
        window_length = max_length

    overlap_factor = 0.95
    overlap_length = int(round(window_length * overlap_factor))
    if overlap_length >= window_length:
        overlap_length = window_length - 1

    pad_factor = 4
    fft_points = pad_factor * window_length

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
    data_spec_MTI2 = np.zeros((len(f), len(t)))

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
        data_spec_MTI2 += np.abs(Sxx)

    data_spec_MTI2 /= np.max(np.abs(data_spec_MTI2))
    data_spec_MTI2 = 20 * np.log10(data_spec_MTI2 + 1e-10)
    data_spec_MTI2 = (data_spec_MTI2 + 100) * 0.3 + 100
    velocity_axis = f * 3e8 / (2 * fc)

    # Plot
    plt.figure(figsize=(8, 6))
    plt.imshow(data_spec_MTI2,
               extent=[t[0], t[-1], velocity_axis[0], velocity_axis[-1]],
               aspect='auto', cmap='jet')
    plt.ylim([velocity_axis[0], velocity_axis[-1]])
    plt.xlabel('Time [s]')
    plt.ylabel('Velocity [m/s]')
    plt.title('Spectrogram of Radar Signal')
    plt.colorbar(label='Intensity (dB)')
    plt.show()

    plt.pcolormesh(t, f, Sxx, shading='gouraud')
    plt.ylabel('Frequency [Hz]')
    plt.xlabel('Time [sec]')
    plt.show()

# 실행
process_radar_data("/home/juhun/Downloads/random_1900_2200_4096.txt")

end_time = time.time()
elapsed_time = end_time - start_time
print(f"실행 시간: {elapsed_time:.4f}초")
