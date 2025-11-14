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
LOOP_HZ = 60
LOOP_DT = 1.0 / LOOP_HZ

# Servo IDs
SERVO_ID1 = 1   # base rotation (yaw)
SERVO_ID2 = 2
SERVO_ID3 = 3
SERVO_ID4 = 4   # head pitch
SERVO_ID5 = 5
SERVO_ID6 = 6   # gripper

DEFAULT_SPEED = 1500
DEFAULT_ACCEL = 255

# Initial positions
init_pose = [1989, 2144, 1048, 1021, 1129, 2040]

# Servo limits
MIN_MOTOR_ID1 = 750
MAX_MOTOR_ID1 = 3278

MIN_MOTOR_ID4 = 1639
MAX_MOTOR_ID4 = 795

MIN_MOTOR_ID6 = 2040
MAX_MOTOR_ID6 = 3432

# ================= FUNCTIONS =================
def map_value(x, in_min, in_max, out_min, out_max):
    x = max(min(x, in_max), in_min)
    return int(out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min))

def apply_deadzone(value, threshold=2.0):
    return 0 if abs(value) < threshold else value

def get_mouth_opening(shape):
    top = shape.part(62)
    bottom = shape.part(66)
    chin = shape.part(8)
    nose = shape.part(27)
    mouth_dist = math.hypot(top.x - bottom.x, top.y - bottom.y)
    face_height = math.hypot(chin.x - nose.x, chin.y - nose.y)
    return (mouth_dist / face_height) * 80.0

def get_head_pitch(shape, frame_h):
    nose_y = shape.part(30).y
    center_y = frame_h / 2
    return (nose_y - center_y) / frame_h * 80.0

def get_head_yaw(shape, frame_w):
    nose_x = shape.part(30).x
    center_x = frame_w / 2
    return (nose_x - center_x) / frame_w * 50.0

# ================= SERVO SETUP =================
portHandler = PortHandler(PORT)
packetHandler = sms_sts(portHandler)

if not portHandler.openPort():
    sys.exit("Failed to open port")
if not portHandler.setBaudRate(BAUD):
    sys.exit("Failed to set baudrate")

print("Port opened successfully")

# Move to initial pose
servo_ids = [SERVO_ID1, SERVO_ID2, SERVO_ID3, SERVO_ID4, SERVO_ID5, SERVO_ID6]
print("Moving servos to initial positions...")
for i, sid in enumerate(servo_ids):
    packetHandler.WritePosEx(sid, init_pose[i], DEFAULT_SPEED, DEFAULT_ACCEL)
    print(f"  Servo {sid} → {init_pose[i]}")
    time.sleep(0.02)
print("Done.\n")

# ================= DLIB SETUP =================
predictor_path = "/home/panha/LeRobot-SO101-simulation-and-hardware/src/data/shape_predictor_68_face_landmarks.dat"
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

print("Starting Dlib face tracking + servo control @ 60 Hz (ESC to exit)")

# ================= MAIN LOOP =================
while True:
    loop_start = time.time()

    ret, frame = cap.read()
    if not ret:
        print("Camera read failed")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = detector(gray, 0)

    cv2.putText(frame, f"Faces: {len(faces)}", (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    if len(faces) == 0:
        cv2.putText(frame, "No face detected", (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    for face in faces:
        x, y, w, h = face.left(), face.top(), face.width(), face.height()
        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

        shape = predictor(gray, face)

        # Calculate
        mouth_open = get_mouth_opening(shape)
        pitch = apply_deadzone(get_head_pitch(shape, frame.shape[0]))
        yaw   = apply_deadzone(get_head_yaw(shape, frame.shape[1]))

        # ====== SERVO OUTPUTS (same as your original) ======
        gripper_pos = map_value(mouth_open, 5, 25, MIN_MOTOR_ID6, MAX_MOTOR_ID6)
        wrist_pos   = map_value(pitch, -10, 10, MIN_MOTOR_ID4, MAX_MOTOR_ID4)
        base_pos    = map_value(yaw, -15, 15, MIN_MOTOR_ID1, MAX_MOTOR_ID1)

        packetHandler.WritePosEx(SERVO_ID6, gripper_pos, DEFAULT_SPEED, DEFAULT_ACCEL)
        packetHandler.WritePosEx(SERVO_ID4, wrist_pos, DEFAULT_SPEED, DEFAULT_ACCEL)
        packetHandler.WritePosEx(SERVO_ID1, base_pos, DEFAULT_SPEED, DEFAULT_ACCEL)

        # ====== Your print stays EXACTLY the same ======
        print(f"Mouth: {mouth_open:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}, "
              f"Servo6: {gripper_pos}, Servo4: {wrist_pos}, Servo1: {base_pos}")

        # Draw landmarks
        for n in range(68):
            px, py = shape.part(n).x, shape.part(n).y
            cv2.circle(frame, (px, py), 1, (0, 255, 0), -1)

        cv2.putText(frame, f"Mouth: {mouth_open:.2f}", (10, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
        cv2.putText(frame, f"Pitch: {pitch:.1f}", (10, 110),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
        cv2.putText(frame, f"Yaw: {yaw:.1f}", (10, 140),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

    cv2.imshow("Dlib Face Tracking + Servo Control (60 Hz)", frame)
    if cv2.waitKey(1) == 27:
        print("Exiting...")
        break

    # ---- 60 Hz rate limiter ----
    elapsed = time.time() - loop_start
    sleep_time = LOOP_DT - elapsed
    if sleep_time > 0:
        time.sleep(sleep_time)

# ================= CLEANUP =================
cap.release()
cv2.destroyAllWindows()
portHandler.closePort()
print("Camera closed, port closed")
