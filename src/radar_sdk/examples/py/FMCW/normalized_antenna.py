import pprint
import numpy as np
import os
from datetime import datetime
from ifxradarsdk import get_version
from ifxradarsdk.fmcw import DeviceFmcw
from ifxradarsdk.fmcw.types import create_dict_from_sequence
from ifxradarsdk.fmcw.types import FmcwSimpleSequenceConfig, FmcwSequenceChirp

def get_unique_folder(base_path):
    """새 실행 시마다 새로운 폴더 생성"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    folder_path = os.path.join(base_path, timestamp)
    os.makedirs(folder_path, exist_ok=True)
    return folder_path

config = FmcwSimpleSequenceConfig(
    frame_repetition_time_s=21.325e-3,
    chirp_repetition_time_s=500e-6,
    num_chirps=32,
    tdm_mimo=False,
    chirp=FmcwSequenceChirp(
        start_frequency_Hz=59e9,
        end_frequency_Hz=61e9,
        sample_rate_Hz=1e6,
        num_samples=128,
        rx_mask=1,
        tx_mask=1,
        tx_power_level=31,
        lp_cutoff_Hz=500000,
        hp_cutoff_Hz=80000,
        if_gain_dB=30,
    ),
)

with DeviceFmcw() as device:
    sequence = device.create_simple_sequence(config)
    device.set_acquisition_sequence(sequence)

    base_path = "/home/juhun/juhunsnote/uptest"
    session_folder = get_unique_folder(base_path)
    
    # 안테나별 파일을 미리 열어두기
    antenna_files = {}
    for iAnt in range(3):  # 최대 4개 안테나 지원 가능
        file_path = os.path.join(session_folder, f"antenna_{iAnt}.txt")
        antenna_files[iAnt] = open(file_path, "w")  # 파일 초기화

    for frame_number in range(285):
        frame_contents = device.get_next_frame()

        for frame in frame_contents:
            num_rx = np.shape(frame)[0]
            
            for iAnt in range(num_rx):
                mat = frame[iAnt, :, :]
                mat_normalized = ((mat + 1) / 2 * 4095).astype(int)
                mat_flattened = mat_normalized.flatten()
                
                # 프레임 번호와 데이터 추가
                file = antenna_files[iAnt]
                file.write(f"Frame {frame_number}:\n")
                for value in mat_flattened:
                    file.write(f"{value}\n")

    # 파일 닫기
    for file in antenna_files.values():
        file.close()
