#레이더와 카메라의 정보를 수집하고 지정 경로에 저장하는 코드
#레이더느 128프레임 카메라는 5FPS로 정보저장

import os
import time
import cv2
import numpy as np
import threading
from ifxradarsdk.fmcw import DeviceFmcw
from ifxradarsdk.fmcw.types import FmcwSimpleSequenceConfig, FmcwSequenceChirp

# 저장할 폴더 경로 설정
save_dir = "/home/juhun/juhunsnote/uptest"
action_name = 'up'
os.makedirs(save_dir, exist_ok=True)

# 세션 번호 자동 증가 함수
def get_next_session_index(save_dir, action_name):
    max_index = 0
    for filename in os.listdir(save_dir):
        if filename.startswith(action_name):
            parts = filename.replace(".jpg", "").split("_")
            if len(parts) >= 2 and parts[1].isdigit():
                max_index = max(max_index, int(parts[1]))
        elif filename.endswith(".txt") and f"_{action_name}_" in filename:
            parts = filename.replace(".txt", "").split("_")
            if len(parts) >= 3 and parts[2].isdigit():
                max_index = max(max_index, int(parts[2]))
    return max_index + 1

# 세션 ID 할당
session_id = get_next_session_index(save_dir, action_name)

# 중복되지 않는 파일명 생성 함수 (이미지)
def get_unique_filename_jpg(directory, action_name, session_id, extension=".jpg"):
    counter = 1
    filename = f"{action_name}_{session_id}{extension}"
    while os.path.exists(os.path.join(directory, filename)):
        filename = f"{action_name}_{session_id}_{counter}{extension}"
        counter += 1
    return os.path.join(directory, filename)

# 카메라 영상 촬영 및 저장 함수
def capture_camera(cap_, fps=5, duration=6):
    cap = cap_
    if not cap.isOpened():
        return

    start_time = time.time()
    frame_interval = 1.0 / fps
    last_saved_time = start_time  

    print(f"카메라 시작 시간: {time.time()}")
    while True:
        ret, frame = cap.read()

        if not ret or frame is None:
            break

        current_time = time.time()
        elapsed_time = current_time - start_time

        if current_time - last_saved_time >= frame_interval:
            last_saved_time = current_time
            filename = get_unique_filename_jpg(save_dir, action_name, session_id)
            cv2.imwrite(filename, frame)

        if elapsed_time >= duration or (cv2.waitKey(1) & 0xFF == 27):
            break

    cap.release()
    print(f"카메라 종료 시간: {time.time()}")

# 레이더 데이터 수집 함수
def collect_radar_data():
    print(f"레이더 시작 시간: {time.time()}")
    config = FmcwSimpleSequenceConfig(
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

    with DeviceFmcw() as device:
        sequence = device.create_simple_sequence(config)
        device.set_acquisition_sequence(sequence)
        antenna_files = {}

        for frame_number in range(128):
            frame_contents = device.get_next_frame()

            for frame in frame_contents:
                num_rx = np.shape(frame)[0]

                for iAnt in range(num_rx):
                    mat = frame[iAnt, :, :]
                    mat_normalized = ((mat + 1) / 2 * 4095).astype(int)
                    mat_flattened = mat_normalized.flatten()

                    file_path = os.path.join(save_dir, f"{iAnt}_{action_name}_{session_id}.txt")
                    if iAnt not in antenna_files:
                        antenna_files[iAnt] = open(file_path, "w")

                    for value in mat_flattened:
                        antenna_files[iAnt].write(f"{value}\n")

        for file in antenna_files.values():
            file.close()
    print(f"레이더 종료 시간: {time.time()}")
    print("✅ 레이더 데이터 수집 완료!")

# 멀티스레딩을 이용한 동시 실행
if __name__ == "__main__":
    cap_ = cv2.VideoCapture(2, cv2.CAP_V4L2)
    cap_.set(cv2.CAP_PROP_FRAME_WIDTH, 720)
    cap_.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    thread_camera = threading.Thread(target=capture_camera, args=(cap_, 5, 3))
    thread_radar = threading.Thread(target=collect_radar_data)

    thread_camera.start()
    thread_radar.start()

    thread_camera.join()
    thread_radar.join()

    print("✅ 모든 작업이 완료되었습니다!")