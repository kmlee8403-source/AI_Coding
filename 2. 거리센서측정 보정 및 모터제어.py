import time
from buildhat import DistanceSensor, Motor

motor = Motor('A')
dist = DistanceSensor('C')

last_valid_distance = None  # 마지막으로 정상 인식된 거리값 저장
motor_state = None  # 현재 모터 상태("run" 또는 "stop")

while True:
    distance = dist.get_distance()  # 거리(mm 단위)

    # 센서가 값을 잘못 읽었을 때 (-1 또는 None)
    if distance is None or distance < 0:
        print("⚠️ 거리 인식 오류 - 마지막 정상값 유지 중")
        distance = last_valid_distance  # 이전 정상값으로 유지
    else:
        last_valid_distance = distance  # 정상값이면 갱신

    # 거리값이 존재할 경우에만 제어
    if distance is not None:
        if distance < 100:
            if motor_state != "stop":
                motor.stop()
                motor_state = "stop"
                print(f"{distance} mm → stop")
        else:
            if motor_state != "run":
                motor.start(speed=50)
                motor_state = "run"
                print(f"{distance} mm → running")
    else:
        # 아직 한 번도 정상값이 없을 경우
        print("⏳ 거리 데이터 없음, 대기 중...")

    time.sleep(0.2)
