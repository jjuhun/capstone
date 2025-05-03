import os
import time
import cv2
import numpy as np

# 저장할 폴더 경로 설정
save_dir = "/home/juhun/juhunsnote/test"
action_name = 'up'

# 폴더가 존재하지 않으면 생성
if not os.path.exists(save_dir):
    os.makedirs(save_dir)

# 중복되지 않는 파일명 생성 함수
def get_unique_filename_jpg(directory, base_filename, extension=".jpg"):
    counter = 1
    filename = f"{base_filename}{extension}"
    
    while os.path.exists(os.path.join(directory, filename)):
        filename = f"{base_filename}_{counter}{extension}"
        counter += 1

    return os.path.join(directory, filename)

def capture_camera(fps=5, duration=6):  # FPS 및 실행 시간 설정
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)


    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 720)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    start_time = time.time()
    frame_interval = 1.0 / fps  # 1초당 프레임 간격 계산
    last_saved_time = start_time  

    print(f"카메라 자동 중. {duration}초 동안 실행, FPS: {fps}")

    while True:
        ret, frame = cap.read()

        cv2.imshow('Logitech StreamCam', frame)
        current_time = time.time()
        elapsed_time = current_time - start_time


        # 1초에 5번 저장
        if current_time - last_saved_time >= frame_interval:
            last_saved_time = current_time  # 마지막 저장 시간 업데이트
            filename = get_unique_filename_jpg(save_dir, action_name)
            save_path = os.path.join(save_dir, filename)
            cv2.imwrite(save_path, frame)
            print(f"📸 {filename} 저장 완료!")  # 저장된 파일명 출력

        # 6초 후 자동 종료
        if elapsed_time >= duration:
            break
        # ESC 종료
        if cv2.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()

# ✅ FPS 5, 8초 동안 실행
capture_camera(fps=5, duration=6)
