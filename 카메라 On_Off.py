from buildhat import ForceSensor
from picamera2 import Picamera2
import time

# 센서, 카메라 객체 생성
force = ForceSensor('C')
picam2 = Picamera2()

# 상태 플래그
camera_on = False
pressed = False

print("힘센서를 눌러 카메라를 켜고, 다시 누르면 꺼지게 해보세요!")

while True:
    f = force.get_force()
    
    # 일정 이상의 힘을 누르면 토글
    if f > 5 and not pressed:
        pressed = True
        camera_on = not camera_on  # 상태 반전

        if camera_on:
            print("📸 카메라 ON")
            picam2.start_preview()  # 미리보기 시작
            picam2.start()
        else:
            print("🛑 카메라 OFF")
            picam2.stop_preview()
            picam2.stop()

    # 손을 떼면 다시 입력 가능
    if f < 3:
        pressed = False

    time.sleep(0.1)
