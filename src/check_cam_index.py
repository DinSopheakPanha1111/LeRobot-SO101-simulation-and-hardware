#!/usr/bin/env python3
import cv2

print("Scanning for connected cameras...")
for i in range(10):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        print(f"Camera index {i} works")
        cap.release()
    else:
        print(f"Camera index {i} not available")
