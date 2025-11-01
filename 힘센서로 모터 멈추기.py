import time
from buildhat import ForceSensor, Motor

motor = Motor('A')
force = ForceSensor('C')

while True:
    value = force.get_force()
    
    if value > 50:
        motor.stop()
        print("힘 센서 눌림 → 멈춤!")
    else:
        motor.start(50)
        print("안 눌림 → 계속 움직임")
    
    time.sleep(0.2)
