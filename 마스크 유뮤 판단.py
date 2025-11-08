from picamera2 import Picamera2
import cv2, time, threading, csv, datetime, traceback
from buildhat import Motor, ForceSensor, DistanceSensor
from pycoral.utils.edgetpu import make_interpreter
from pycoral.adapters import classify, common

# =========================
# 설정값
# =========================
MODEL_PATH = "/home/pi/converted_edgetpu/model_edgetpu.tflite"
LABEL_PATH = "/home/pi/converted_edgetpu/labels.txt"
CONFIDENCE_THRESHOLD = 0.7
DEBOUNCE_DELAY = 1.0
MOTOR_HOME_ANGLE = 0
MOTOR_OPEN_ANGLE = 90
ANGLE_TOLERANCE = 2

# =========================
# 초기화
# =========================
motorA = Motor('A')  # 정방향 모터
motorB = Motor('B')  # 반대방향 모터
force = ForceSensor('C')
dist_sensor = DistanceSensor('D')

# 엔코더 초기 위치 설정 (부팅 시 기준점 0도)
motorA.run_to_position(0, 50)
motorB.run_to_position(0, 50)

interpreter = make_interpreter(MODEL_PATH)
interpreter.allocate_tensors()

with open(LABEL_PATH, "r") as f:
    labels = [line.strip() for line in f.readlines()]

picam2 = Picamera2()
config = picam2.create_video_configuration(main={"size": (640, 480)})
picam2.configure(config)

# =========================
# 상태 변수
# =========================
last_stable_label = None
potential_label = None
potential_label_start_time = 0
camera_active = False
current_angle = MOTOR_HOME_ANGLE
motor_lock = threading.Lock()  # 모터 동시 명령 방지용 Lock

# =========================
# 모터 제어 함수 (절대 위치 기반)
# =========================
def move_motor_to(target_angle, speed=20):
    """모터 절대 위치 기반 이동"""
    global current_angle
    try:
        current_angle_A = motorA.get_aposition()
        current_angle_B = motorB.get_aposition()

        diff_A = target_angle - current_angle_A
        diff_B = -target_angle - current_angle_B  # 반대 방향

        if abs(diff_A) <= ANGLE_TOLERANCE and abs(diff_B) <= ANGLE_TOLERANCE:
            return

        with motor_lock:
            motorA.run_to_position(target_angle, speed)
            time.sleep(0.3)
            motorB.run_to_position(-target_angle, speed)
            current_angle = target_angle

        if abs(current_angle - MOTOR_HOME_ANGLE) <= ANGLE_TOLERANCE:
            print("문 닫힘 (0도)")
        elif abs(current_angle - MOTOR_OPEN_ANGLE) <= ANGLE_TOLERANCE:
            print("문 열림 (90도)")

    except Exception as e:
        print(f"[모터 이동 오류] {e}")
        traceback.print_exc()

def motor_correction():
    """모터 절대 위치 보정"""
    try:
        with motor_lock:
            posA_before = motorA.get_aposition()
            posB_before = motorB.get_aposition()

            # 오차가 있으면 보정
            if abs(posA_before - current_angle) > ANGLE_TOLERANCE:
                motorA.run_to_position(current_angle, 20)
            if abs(posB_before + current_angle) > ANGLE_TOLERANCE:
                motorB.run_to_position(-current_angle, 20)

            # 보정 후 다시 읽기
            time.sleep(0.5)
            posA_after = motorA.get_aposition()
            posB_after = motorB.get_aposition()

            print(f"보정 완료 (A:{posA_before:.1f}° → {posA_after:.1f}°, "
                  f"B:{posB_before:.1f}° → {posB_after:.1f}°)")

    except Exception as e:
        print(f"[보정 오류] {e}")
        traceback.print_exc()

# =========================
# 로그 기록
# =========================
def log_event(label, distance, score):
    try:
        with open("/home/pi/converted_edgetpu/event_log.csv", "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([datetime.datetime.now(), label, distance, score])
    except Exception as e:
        print(f"[로그 오류] {e}")

# =========================
# 메인 루프
# =========================
try:
    print("시스템 준비 중...")

    # --- 시작 시 0도 위치로 이동 ---
    move_motor_to(MOTOR_HOME_ANGLE, 50)
    time.sleep(1)
    motor_correction()
    print("모터 초기 위치 보정 완료. 힘센서를 눌러 카메라를 켜세요.")

    while True:
        distance = dist_sensor.get_distance()

        # --- 힘센서로 카메라 ON/OFF 토글 ---
        if force.is_pressed():
            camera_active = not camera_active
            if camera_active:
                print("카메라 활성화!")
                picam2.start()
                time.sleep(0.5)
            else:
                print("카메라 비활성화!")
                picam2.stop()
                cv2.destroyAllWindows()
            time.sleep(1)

        # --- 카메라 동작 중일 때 ---
        if camera_active:
            frame = picam2.capture_array()
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_RGBA2RGB)
            frame_rgb = cv2.rotate(frame_rgb, cv2.ROTATE_180)

            # TPU 추론
            tpu_input = cv2.resize(frame_rgb.copy(), common.input_size(interpreter))
            common.set_input(interpreter, tpu_input)
            interpreter.invoke()

            classes = classify.get_classes(interpreter, top_k=1, score_threshold=CONFIDENCE_THRESHOLD)
            current_label = labels[classes[0].id] if classes else None
            current_score = classes[0].score if classes else 0.0

            # --- Debounce 처리 ---
            if current_label != potential_label:
                potential_label = current_label
                potential_label_start_time = time.time()

            if time.time() - potential_label_start_time > DEBOUNCE_DELAY:
                if potential_label != last_stable_label:
                    last_stable_label = potential_label
                    log_event(last_stable_label, distance, current_score)

                    def motor_action(label):
                        try:
                            if label == labels[0]:
                                move_motor_to(MOTOR_OPEN_ANGLE)
                            elif label == labels[1]:
                                move_motor_to(MOTOR_HOME_ANGLE)
                            time.sleep(0.5)
                            motor_correction()
                        except Exception as e:
                            print(f"[모터 실행 오류] {e}")
                            traceback.print_exc()

                    threading.Thread(target=motor_action, args=(last_stable_label,)).start()

            # --- 영상 표시 ---
            display_text1 = f"Label: {last_stable_label or 'None'} | Score: {current_score:.2f}"
            display_text2 = f"Distance: {distance:.1f} mm"
            cv2.putText(frame_rgb, display_text1, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame_rgb, display_text2, (10, 450),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)

            frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
            cv2.imshow("Edge TPU Door Control", frame_bgr)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("프로그램 종료 요청")
                break

        time.sleep(0.1)

except KeyboardInterrupt:
    print("사용자 중단")

finally:
    try:
        motorA.stop()
        motorB.stop()
        picam2.stop()
        cv2.destroyAllWindows()
    except:
        pass
    print("정상 종료 완료.")


