#!/usr/bin/env python3
import sys, os, time, math
import cv2
import dlib
import numpy as np
sys.path.append("..")
from scservo_sdk import *

# ================= CONFIG =================
PORT = '/dev/ttyACM0'
BAUD = 1000000

# Servo IDs
SERVO_ID1 = 1   # base rotation (yaw)
SERVO_ID2 = 2
SERVO_ID3 = 3
SERVO_ID4 = 4   # head up/down (pitch)
SERVO_ID5 = 5
SERVO_ID6 = 6   # gripper open/close

DEFAULT_SPEED = 1500
DEFAULT_ACCEL = 255

# Initial positions
init_pose = [2030, 3165, 0, 2300, 3590, 1940]

# Servo limits
MIN_MOTOR_ID1 = 800
MAX_MOTOR_ID1 = 2800
MIN_MOTOR_ID4 = 2200
MAX_MOTOR_ID4 = 3005
MIN_MOTOR_ID6 = 1940
MAX_MOTOR_ID6 = 3023

# Low-pass filter constants (0.0–1.0, lower = smoother)
ALPHA_YAW   = 0.2
ALPHA_PITCH = 0.2
ALPHA_MOUTH = 0.3

# ================= FUNCTIONS =================
def map_value(x, in_min, in_max, out_min, out_max):
    x = max(min(x, in_max), in_min)
    return int(out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min))

def apply_deadzone(value, threshold=2.0):
    return 0 if abs(value) < threshold else value

def lowpass_filter(prev, new, alpha):
    return alpha * new + (1 - alpha) * prev

def get_mouth_opening(shape):
    top = shape.part(62)
    bottom = shape.part(66)
    chin = shape.part(8)
    nose = shape.part(27)
    mouth_dist = math.hypot(top.x - bottom.x, top.y - bottom.y)
    face_height = math.hypot(chin.x - nose.x, chin.y - nose.y)
    return (mouth_dist / face_height) * 155.0  # normalized %

def get_head_pitch(shape, frame_h):
    nose_y = shape.part(30).y
    center_y = frame_h / 2
    return (center_y - nose_y) / frame_h * 150.0

def get_head_yaw(shape, frame_w):
    nose_x = shape.part(30).x
    center_x = frame_w / 2
    return (nose_x - center_x) / frame_w * 100.0

def init_position(packetHandler):
    print("Moving all servos to initial position...")
    for sid, pos in zip(range(1, 7), init_pose):
        packetHandler.WritePosEx(sid, pos, DEFAULT_SPEED, DEFAULT_ACCEL)
        print(f"  Servo {sid} → {pos}")
        time.sleep(0.02)
    print("All servos commanded to initial positions.\n")

# ================= SERVO SETUP =================
portHandler = PortHandler(PORT)
packetHandler = sms_sts(portHandler)

if not portHandler.openPort():
    sys.exit("Failed to open port")
if not portHandler.setBaudRate(BAUD):
    sys.exit("Failed to set baudrate")

print("Port opened successfully")
init_position(packetHandler)
time.sleep(2)

# ================= DLIB SETUP =================
predictor_path = "data/shape_predictor_68_face_landmarks.dat"
if not os.path.exists(predictor_path):
    sys.exit(f"Missing predictor file: {predictor_path}")
detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor(predictor_path)

# ================= CAMERA =================
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
if not cap.isOpened():
    sys.exit("Cannot open camera")

print("🎥 Starting Dlib face control... (Press ESC to exit)")

# Initialize filtered values
filt_yaw = filt_pitch = filt_mouth = 0.0

# ================= MAIN LOOP =================
while True:
    ret, frame = cap.read()
    if not ret:
        print("Camera read failed")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = detector(gray, 0)
    cv2.putText(frame, f"Faces detected: {len(faces)}", (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    if len(faces) == 0:
        cv2.putText(frame, "No face detected", (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    for face in faces:
        shape = predictor(gray, face)

        # --- RAW VALUES ---
        mouth_open = get_mouth_opening(shape)
        pitch_raw  = get_head_pitch(shape, frame.shape[0])
        yaw_raw    = get_head_yaw(shape, frame.shape[1])

        # --- FILTERED VALUES ---
        filt_mouth = lowpass_filter(filt_mouth, mouth_open, ALPHA_MOUTH)
        filt_pitch = lowpass_filter(filt_pitch, pitch_raw,  ALPHA_PITCH)
        filt_yaw   = lowpass_filter(filt_yaw,   yaw_raw,    ALPHA_YAW)

        # ======= GRIPPER (Servo 6) =======
        gripper_pos = map_value(filt_mouth, 5, 25, MIN_MOTOR_ID6, MAX_MOTOR_ID6)
        packetHandler.WritePosEx(SERVO_ID6, gripper_pos, DEFAULT_SPEED, DEFAULT_ACCEL)

        # ======= HEAD UP/DOWN (Servo 4) =======
        pitch = apply_deadzone(filt_pitch, 2.0)
        wrist_pos = map_value(pitch, -10, 10, MIN_MOTOR_ID4, MAX_MOTOR_ID4)
        packetHandler.WritePosEx(SERVO_ID4, wrist_pos, DEFAULT_SPEED, DEFAULT_ACCEL)

        # ======= HEAD LEFT/RIGHT (Servo 1) =======
        yaw = apply_deadzone(filt_yaw, 2.0)
        base_pos = map_value(yaw, -15, 15, MIN_MOTOR_ID1, MAX_MOTOR_ID1)
        packetHandler.WritePosEx(SERVO_ID1, base_pos, DEFAULT_SPEED, DEFAULT_ACCEL)

        print(f"➡️ Mouth:{filt_mouth:.1f}  Pitch:{filt_pitch:.1f}  Yaw:{filt_yaw:.1f}  "
              f"S6:{gripper_pos} S4:{wrist_pos} S1:{base_pos}")

        # Draw landmarks
        for n in range(68):
            x, y = shape.part(n).x, shape.part(n).y
            cv2.circle(frame, (x, y), 1, (0, 255, 0), -1)

        cv2.putText(frame, f"Mouth: {filt_mouth:.1f}", (10, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
        cv2.putText(frame, f"Pitch: {filt_pitch:.1f}", (10, 110),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
        cv2.putText(frame, f"Yaw: {filt_yaw:.1f}", (10, 140),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

    cv2.imshow("Dlib Face Control (Yaw + Pitch + Mouth)", frame)
    if cv2.waitKey(1) == 27:  # ESC
        print("Exiting by user")
        break

# ================= CLEANUP =================
cap.release()
cv2.destroyAllWindows()
portHandler.closePort()
print("Port closed cleanly")
