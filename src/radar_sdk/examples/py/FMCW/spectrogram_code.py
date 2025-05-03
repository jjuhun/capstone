# 파일 및 시스템 관련
import os
import glob
import datetime
import random as rn

# 수치 및 데이터 처리
import numpy as np
import cv2
from tqdm import tqdm

# 신호 처리
from scipy import signal

# 데이터 전처리 (Scikit-learn)
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import LabelEncoder

# 시각화
import matplotlib.pyplot as plt

# 시드값 고정
seed_num = 42
os.environ['PYTHONHASHSEED'] = str(seed_num)
os.environ['TF_DETERMINISTIC_OPS'] = '1'
os.environ['TF_NUM_INTRAOP_THREADS'] = '1'
os.environ['TF_NUM_INTEROP_THREADS'] = '1'

# 모든 시드 설정
np.random.seed(seed_num)
rn.seed(seed_num)

# 섹션1. 데이터 준비
def odd_number(n):
   """가장 가까운 홀수를 반환하는 함수"""
   return n if n % 2 == 1 else n + 1

def read_real_data(filename):
    """실수 형태의 레이더 데이터를 읽는 함수"""
    rx1 = np.loadtxt(filename)  # 1열 데이터만 포함된 txt 파일 로드
    data = rx1

    fc = 61.25e9        # 중심주파수 61.25 GHz
    Tsweep = 250.15e-6  # 스윕 시간 250.15 μs
    NTS = 128        # 스윕당 샘플 수
    Bw = 0.7e9         # 대역폭 0.7 GHz
    header = np.array([fc, Tsweep, NTS, Bw])

    return header, data

def extract_label_from_path(file_path):
    """
    파일 경로에서 레이블을 추출하는 함수
    
    Args:
        file_path: 레이더 데이터 파일 경로
        
    Returns:
        label: 추출된 레이블 (폴더명 기반)
    
    경로 형식: dataset/250309/sit/121234.txt
    레이블은 디렉토리 이름(sit, walk 등)에서 직접 추출
    """
    # 디렉토리 경로 구하기
    directory = os.path.dirname(file_path)  # dataset/250309/sit
    label = os.path.basename(directory)     # sit
    
    return label

# 섹션2. 레이더 신호 처리
def process_radar_data(filename, include_labels=False, visualize=False):
    """
    레이더 데이터를 처리하여 마이크로 도플러 시그니처(스펙트로그램)를 생성하는 함수
    
    Args:
        filename: 레이더 데이터 파일 경로
        include_labels: 라벨과 축 정보를 포함할지 여부 (시각화용 True, 융합용 False)
        visualize: 이미지를 시각화할지 여부
        
    Returns:
        spectrogram_img: 생성된 스펙트로그램 이미지 (라벨 없음)
    """
    # 레이더 데이터 읽기
    header, data = read_real_data(filename) 
    
    # 매개변수 추출
    fc = header[0]  # 중심주파수
    Tsweep = header[1]  # 스윕 시간 (초)
    NTS = int(header[2])  # 스윕당 시간 샘플 수
    Bw = header[3]  # FMCW 대역폭
    
    # 파생 매개변수 계산
    record_length = len(data) / NTS * Tsweep  # 녹화 길이 (초)
    nc = int(record_length / Tsweep)  # 처프 수
    
    # 데이터를 처프로 재구성
    data_time = data.reshape((NTS, nc), order='F')
    
    # 사각 윈도우 함수 적용
    win = np.ones((NTS, data_time.shape[1]))
    
    # FFT 처리 (실수 데이터 적용)
    tmp = np.fft.fftshift(np.fft.fft(data_time * win, axis=0), axes=0)
    data_range = tmp[NTS//2:NTS, :] 
    
    # MTI 필터링 (정지된 배경 신호 제거)
    ns = odd_number(data_range.shape[1]) - 1
    data_range_MTI = np.zeros((data_range.shape[0], ns))
    
    # Butterworth 필터 설계 및 적용 (저주파 노이즈 제거)
    b, a = signal.butter(4, 0.0075, 'high')
    for k in range(data_range.shape[0]):
        data_range_MTI[k, :] = signal.lfilter(b, a, np.real(data_range[k, :ns]))
    
    # 첫 번째 거리 빈 제거
    data_range_MTI = data_range_MTI[1:, :]
    data_range = data_range[1:, :]

    # 스펙트로그램 처리 매개변수
    bin_indl = 10   # 2.14m (bin_indl * c/2B)
    bin_indu = 30   # 6.42m (bin_indu * c/2B)
    PRF = 1/Tsweep
    window_length = 200
    overlap_factor = 0.95
    overlap_length = int(round(window_length * overlap_factor))
    pad_factor = 4
    fft_points = pad_factor * window_length
    
    # 첫 bin으로 축 정보 추출
    f, t, test_Sxx = signal.spectrogram(
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
    data_spec_MTI2 = np.zeros_like(test_Sxx)
    
    # 스펙트로그램 계산
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
    
    # 전체 데이터 정규화 및 스케일링
    data_max = np.max(np.abs(data_spec_MTI2))
    if data_max > 0:
        data_spec_MTI2 = data_spec_MTI2 / data_max
    data_spec_MTI2 = 20 * np.log10(data_spec_MTI2 + 1e-10)
    data_spec_MTI2 = (data_spec_MTI2 + 100) * 0.3 + 100
    
    # 도플러 주파수 → 속도 변환
    velocity_axis = f * 3e8 / (2 * fc)
    
    # 파일명 추출 (제목용)
    base_filename = os.path.basename(filename)
    
    # 시각화 (라벨 포함)
    if visualize and include_labels:
        plt.figure(figsize=(8, 6))
        plt.imshow(data_spec_MTI2,
                   extent=[t[0], t[-1], velocity_axis[0], velocity_axis[-1]],
                   aspect='auto', cmap='jet')
        plt.ylim([velocity_axis[0], velocity_axis[-1]])   # 화면에 가득 차도록 속도 범위를 [0, 15]m/s로 제한
        # plt.clim([100, 130]) # 색상 범위를 [100, 130]으로 제한
        plt.colorbar(label='Intensity (dB)')
        plt.xlabel('Time [s]')
        plt.ylabel('Velocity [m/s]')
        plt.title(f'Spectrogram: {base_filename}')
        plt.show()
        
        plt.pcolormesh(t, f, Sxx, shading='gouraud')
        plt.ylabel('Frequency [Hz]')
        plt.xlabel('Time [sec]')
        plt.show()
    
    # 순수 이미지 생성 (라벨 미포함)
    plt.figure(figsize=(8, 6))
    plt.imshow(data_spec_MTI2,
               extent=[t[0], t[-1], velocity_axis[0], velocity_axis[-1]],
               aspect='auto', cmap='jet')
    plt.ylim([velocity_axis[0], velocity_axis[-1]])
    # plt.clim([100, 130])
    plt.axis('off')  # 축 정보 제거
    
    # 여백 제거
    plt.tight_layout(pad=0)
    plt.subplots_adjust(left=0, right=1, top=1, bottom=0)
    
    # 이미지 데이터 가져오기
    spectrogram_fig = plt.gcf()
    spectrogram_fig.canvas.draw()
    spectrogram_img = np.array(spectrogram_fig.canvas.renderer.buffer_rgba())[:,:,:3]  # RGBA에서 RGB로 변환
    plt.close()
    
    # 크기 조정 (128, 128)로 통일
    spectrogram_img = cv2.resize(spectrogram_img, (128, 128))
    
    return spectrogram_img

if __name__ == "__main__":
    # 새로운 데이터를 사용하는 경우
    data_dir = "dataset/250322"
    
    # 기존에 저장된 데이터 파일을 사용하는 경우
    # data_file = "processed_data/multimodal_data_250411_155654.h5"
    # run_multimodal_comparison_pipeline(data_file=data_file, visualize_samples=1)