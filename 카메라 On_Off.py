from buildhat import ForceSensor
from picamera2 import Picamera2
import cv2, time

# ForceSensor는 Build HAT의 포트 C에 연결했다고 가정
force = ForceSensor('C')
picam2 = Picamera2()

camera_on = False
pressed = False

print("힘센서를 눌러 카메라를 켜고, 다시 누르면 꺼보세요!")

while True:
    f = force.get_force()

    # 힘센서 눌렀을 때 토글
    if f > 5 and not pressed:
        pressed = True
        camera_on = not camera_on

        if camera_on:
            print("📷 카메라 ON")
            picam2.start()
        else:
            print("🛑 카메라 OFF")
            picam2.stop()
            cv2.destroyAllWindows()

    # 손을 떼면 다시 입력 가능
    if f < 3:
        pressed = False

    # 카메라 ON 상태일 때만 화면 표시
    if camera_on:
        frame = picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        cv2.imshow("Camera Preview", frame)

        # 'q' 키를 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("프로그램 종료")
            break

    time.sleep(0.1)

cv2.destroyAllWindows()
