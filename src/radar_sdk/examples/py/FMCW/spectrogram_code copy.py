# Re-import required packages after reset
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
import time

start_time = time.time() #시간 측정()



# Helper functions
def odd_number(n):
   return n if n % 2 == 1 else n + 1

def read_real_data(filename):
    rx1 = np.loadtxt(filename)
    data = rx1
    fc = 61.25e9
    Tsweep = 250.15e-6
    NTS = 128
    Bw = 0.7e9
    header = np.array([fc, Tsweep, NTS, Bw])
    return header, data

# Main radar processing function
def process_radar_data(filename):
    header, data = read_real_data(filename)
    fc, Tsweep, NTS, Bw = header

    nc = int((len(data) / NTS * Tsweep) / Tsweep)
    data_time = data.reshape((int(NTS), int(nc)), order='F')
    win = np.ones((int(NTS), data_time.shape[1]))
    tmp = np.fft.fftshift(np.fft.fft(data_time * win, axis=0), axes=0)
    data_range = tmp[int(NTS)//2:int(NTS), :]
    ns = odd_number(data_range.shape[1]) - 1
    data_range_MTI = np.zeros((data_range.shape[0], ns))

    b, a = signal.butter(4, 0.0075, 'high')
    for k in range(data_range.shape[0]):
        data_range_MTI[k, :] = signal.lfilter(b, a, np.real(data_range[k, :ns]))
    
    
    data_range_MTI = data_range_MTI[1:, :]
    # data_range = data_range[1:, :]
    
    bin_indl, bin_indu = 10, 30
    PRF = 1 / Tsweep
    window_length = 200
    overlap_factor = 0.95
    overlap_length = int(round(window_length * overlap_factor))
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
    data_spec_MTI2 = np.zeros((len(f), len(t))) #여기 유의깊게


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

    # Plotting the spectrogram
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
end_time = time.time()
elapsed_time = end_time - start_time
print(f"실행 시간: {elapsed_time}초")

# Run the function using the uploaded file
process_radar_data("/home/juhun/Downloads/0_run_1_zeroed.txt")
