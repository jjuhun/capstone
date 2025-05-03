import numpy as np
import os
from ifxradarsdk.fmcw import DeviceFmcw
from ifxradarsdk.fmcw.types import FmcwSimpleSequenceConfig, FmcwSequenceChirp

# 저장할 폴더 경로 설정
base_save_dir = "/home/juhun/juhunsnote/uptest"
action_name = 'UP'
os.makedirs(base_save_dir, exist_ok=True)

# 파일명 중복 방지 함수
def get_unique_filename(directory, action_name, extension=".txt"):
    filename = f"{action_name}{extension}"
    counter = 1
    while os.path.exists(os.path.join(directory, filename)):
        filename = f"{action_name}_{counter}{extension}"
        counter += 1
    return os.path.join(directory, filename)

config = FmcwSimpleSequenceConfig(
    frame_repetition_time_s=307.325e-3,
    chirp_repetition_time_s=500e-6,
    num_chirps=32,
    tdm_mimo=False,
    chirp=FmcwSequenceChirp(
        start_frequency_Hz=59e9,
        end_frequency_Hz=61e9,
        sample_rate_Hz=1e6,
        num_samples=128,
        rx_mask=7,
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
    chirp_loop = sequence.loop.sub_sequence.contents
    metrics = device.metrics_from_sequence(chirp_loop)

    antenna_files = {}

    
    for frame_number in range(10):
        frame_contents = device.get_next_frame()

        for frame in frame_contents:
            num_rx = np.shape(frame)[0]

            for iAnt in range(num_rx):
                mat = frame[iAnt, :, :]
                mat_normalized = ((mat + 1) / 2 * 4095).astype(int)
                mat_flattened = mat_normalized.flatten()

                file_path = get_unique_filename(base_save_dir, f"{iAnt}_{action_name}")

                if iAnt not in antenna_files:
                    antenna_files[iAnt] = open(file_path, "w")

                for value in mat_flattened:
                    antenna_files[iAnt].write(f"{value}\n")

    print("✅ 모든 안테나 데이터 저장 완료!")
