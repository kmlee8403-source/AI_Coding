import time
from buildhat import ForceSensor, Motor

motor = Motor('A')       # Motor connected to port A
force = ForceSensor('D') # Force sensor connected to port D

last_valid_force = 0     # Store last valid force reading
motor_state = None       # Track whether the motor is running or stopped

print("Running... (Press Ctrl+C to stop)\n")

while True:
    value = force.get_force()  # Read force value (0–100)

    # Handle invalid or missing readings
    if value is None or value < 0:
        print("⚠️ Sensor read error - keeping last value")
        value = last_valid_force
    else:
        last_valid_force = value

    # Map force directly to motor speed (0–100)
    speed = int(value)

    # Motor control based on force
    if speed > 0:
        motor.start(speed=speed)
        if motor_state != "run":
            motor_state = "run"
        print(f"Force: {value:3} → Motor speed: {speed}")
    else:
        motor.stop()
        if motor_state != "stop":
            motor_state = "stop"
        print(f"Force: {value:3} → Motor stopped")

    time.sleep(0.1)  # Update every 0.1 second
