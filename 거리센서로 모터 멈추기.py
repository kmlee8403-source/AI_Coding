import time
from buildhat import DistanceSensor, Motor

motor = Motor('A')
dist = DistanceSensor('B')

while True:
    distance = dist.get_distance()
    
    if distance < 100:
        motor.stop()
        print("물체가 가까워서 멈춤!")
    else:
        motor.start(50)
        print("안전, 이동 중...")
    
    time.sleep(0.2)  # 0.2초마다 센서값 확인
