#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
test_qr_camera.py
Test camera and QR code detection module
Function: Capture one frame, detect QR code, draw markers, save as test_photo.jpg
"""

import cv2
import numpy as np
from camera_module import CameraModule
from qr_detector import QRDetector


def main():
    # Initialize camera module
    cam = CameraModule()
    cam.open()

    # Initialize QR code detector
    detector = QRDetector()

    # Get one frame (first frame from generator)
    frame_generator = cam.frames()
    frame = next(frame_generator)

    # Detect QR code
    result = detector.detect(frame)

    # Draw markers on image
    h, w = frame.shape[:2]
    font = cv2.FONT_HERSHEY_SIMPLEX

    if result["lost"]:
        # No QR code found: display red text at center
        text = "No QR Code Found"
        text_size = cv2.getTextSize(text, font, 1, 2)[0]
        text_x = (w - text_size[0]) // 2
        text_y = (h + text_size[1]) // 2
        cv2.putText(frame, text, (text_x, text_y), font, 1, (0, 0, 255), 2)
    else:
        # QR code found: draw green bounding box
        pts = np.array(result["points"], dtype=np.int32)
        cv2.polylines(frame, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

        # Mark corners with purple dots and connect them
        for i, point in enumerate(pts):
            cv2.circle(frame, tuple(point), 4, (255, 0, 255), -1)
            # Connect lines
            if i < len(pts) - 1:
                cv2.line(frame, tuple(point), tuple(pts[i + 1]), (255, 0, 255), 2)
            else:
                cv2.line(frame, tuple(point), tuple(pts[0]), (255, 0, 255), 2)

        # Display QR code content and distance at top-left corner
        text_data = f"ID: {result['data']}"
        text_dist = f"Distance: {result['distance_cm']:.1f} cm"
        cv2.putText(frame, text_data, (10, 30), font, 0.7, (0, 0, 255), 2)
        cv2.putText(frame, text_dist, (10, 60), font, 0.7, (0, 0, 255), 2)

    # Save image
    output_path = "test_photo.jpg"
    cv2.imwrite(output_path, frame)
    print(f"Image saved to: {output_path}")

    # Close camera
    cam.close()


if __name__ == "__main__":
    main()