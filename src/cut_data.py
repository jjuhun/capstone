import numpy as np
import os
import glob
import re

# ✅ [1] 데이터셋이 있는 폴더 경로 설정
input_dir = "/home/juhun/capstone_ws/dataset/run"  # 예: "./raw_data", "D:/RadarProject/Input"
input_pattern = os.path.join(input_dir, "0_run_*.txt")

# ✅ [2] 슬라이싱된 데이터를 저장할 폴더 경로 설정
save_dir = "/home/juhun/capstone_ws/dataset/run_crop"  # 예: "./processed_data", "D:/RadarProject/Sliced"
os.makedirs(save_dir, exist_ok=True)

# ✅ [3] 저장 폴더에 이미 존재하는 파일들의 최대 인덱스 확인
existing_files = glob.glob(os.path.join(save_dir, "0_run_*.txt"))
existing_indices = []
for fname in existing_files:
    match = re.match(r"0_run_(\d+)\.txt", os.path.basename(fname))
    if match:
        existing_indices.append(int(match.grorun(1)))

# 저장할 시작 번호
save_index = max(existing_indices, default=0) + 1

# ✅ [4] 입력 파일 리스트 확보 (파일 크기 조건 생략 가능)
input_files = sorted(glob.glob(input_pattern))

for input_file in input_files:
    print(f"{input_file} 처리 중...")

    # 데이터 불러오기
    data = np.loadtxt(input_file)

    # (128, 4096)으로 reshape
    frames = data.reshape((128, 128 * 32))

    # 슬라이딩 윈도우로 100프레임씩 자르기
    for i in range(128 - 100 + 1):  # 29개
        window = frames[i:i + 100]
        window_flat = window.flatten()

        # 파일 저장
        filename = f"0_run_{save_index}.txt"
        save_path = os.path.join(save_dir, filename)
        np.savetxt(save_path, window_flat, fmt='%.6f')
        print(f"저장 완료: {save_path}")

        save_index += 1
