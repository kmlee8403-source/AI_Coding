from picamera2 import Picamera2
import cv2
from buildhat import Motor
from pycoral.utils.edgetpu import make_interpreter
from pycoral.adapters import classify, common
import time

# 설정

MODEL_PATH = "/home/pi/converted_edgetpu/model_edgetpu.tflite"    #tflite 모델의 경로를 지정
LABEL_PATH = "/home/pi/converted_edgetpu/labels.txt"                      #labes 파일의 경로를 지정
CONFIDENCE_THRESHOLD = 0.7
DEBOUNCE_DELAY = 1.0

# 라벨 로드
with open(LABEL_PATH, "r") as f:
    labels = [line.strip() for line in f.readlines()]
# TPU 및 모터 초기화
interpreter = make_interpreter(MODEL_PATH)
interpreter.allocate_tensors()
motor = Motor('A')
# 상태 관리 변수 선언
last_stable_label = None
potential_label = None
potential_label_start_time = 0
# Picamera2 초기화
picam2 = Picamera2()
config = picam2.create_video_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()

print("Starting stable motor control with 70% confidence threshold...")

try:
    while True:
        frame = picam2.capture_array()
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_RGBA2RGB)
        
        tpu_input = cv2.resize(frame_rgb.copy(), common.input_size(interpreter))
        common.set_input(interpreter, tpu_input)
        interpreter.invoke()
        
        classes = classify.get_classes(interpreter, top_k=1, score_threshold=CONFIDENCE_THRESHOLD)
        current_label = labels[classes[0].id] if classes else None

        if current_label != potential_label:
            potential_label = current_label
            potential_label_start_time = time.time()
            print(f"Potential change detected -> '{potential_label}'. Starting debounce timer...")

        if time.time() - potential_label_start_time > DEBOUNCE_DELAY:
            if potential_label != last_stable_label:
                print(f"STABLE STATE CHANGE CONFIRMED: From '{last_stable_label}' to '{potential_label}'")
                last_stable_label = potential_label

                if last_stable_label == labels[0]:
                    print("  -> Action: Motor running for 5s...")
                    motor.run_for_seconds(5, speed=30)
                    print("  -> Action complete.")
                
                elif last_stable_label == labels[1]:
                    print("  -> Action: Motor running for 2s...")
                    motor.run_for_seconds(2, speed=-80)
                    print("  -> Action complete.")

        display_text = f"Stable: {last_stable_label} | Current: {current_label}"
        cv2.putText(frame_rgb, display_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        frame_display_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        cv2.imshow("Edge TPU Stable Control (70%)", frame_display_bgr)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Stopped by user.")

finally:
    motor.stop()
    picam2.stop()
    cv2.destroyAllWindows()
    print("Program finished.")
