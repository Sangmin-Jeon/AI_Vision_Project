import json
import csv
import time
from ultralytics import YOLO
import cv2
import math 
import os
import shutil
import sys
import numpy as np

# 경계선 설정
point1, point2 = None, None
previous_positions = {}

def draw_line(event, x, y, flags, param):
    global point1, point2
    if event == cv2.EVENT_LBUTTONDOWN:
        if point1 is None:
            point1 = (x, y)
        else:
            point2 = (x, y)

def is_crossing_boundary(prev_center, curr_center, p1, p2):
    # 외부에서 내부로 이동하는지 확인
    def side(point, p1, p2):
        return (p2[0] - p1[0]) * (point[1] - p1[1]) - (p2[1] - p1[1]) * (point[0] - p1[0])
    
    prev_side = side(prev_center, p1, p2)
    curr_side = side(curr_center, p1, p2)

    # 이전 위치는 외부에 있고 현재 위치는 내부에 있는지 확인
    return prev_side > 0 and curr_side <= 0

def run_yolo(model, output_dir):
    global img, previous_positions, point1, point2
    cap = cv2.VideoCapture('/dev/video2')  # USB 카메라 사용
    cap.set(3, 640)
    cap.set(4, 480)

    cv2.namedWindow('Webcam')
    cv2.setMouseCallback('Webcam', draw_line)

    alert_triggered = False

    while True:
        ret, img = cap.read()
        if not ret:
            break

        # 경계선 그리기
        if point1 and point2:
            cv2.line(img, point1, point2, (0, 0, 255), 2)  # 빨간색 선

        results = model(img, stream=True)
        object_count = 0  # 물체 카운트 초기화
        classNames = ['car']
        csv_output = []
        confidences = []

        for r in results:
            boxes = r.boxes
            for i, box in enumerate(boxes):
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
                confidence = math.ceil((box.conf[0] * 100)) / 100
                cls = int(box.cls[0])
                confidences.append(confidence)

                org = (x1, y1)
                font = cv2.FONT_HERSHEY_SIMPLEX
                color = (255, 0, 0)
                thickness = 2

                # 탐지된 물체의 이름과 정확도 표시
                cv2.putText(img, f"{classNames[cls]}: {confidence}", org, font, 1, color, thickness)

                center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2
                cv2.circle(img, (center_x, center_y), 5, (0, 255, 255), -1)

                # 이전 위치 저장 및 경계선 넘는지 확인
                object_id = f"obj_{i}"
                current_center = (center_x, center_y)

                if object_id in previous_positions:
                    if point1 and point2:
                        if is_crossing_boundary(previous_positions[object_id], current_center, point1, point2):
                            alert_triggered = True

                previous_positions[object_id] = current_center  # 현재 위치 업데이트
                csv_output.append([x1, y1, x2, y2, confidence, cls])
                object_count += 1

        # 경고 메시지 표시
        if alert_triggered:
            cv2.putText(img, "ALERT: Vehicle entering!", (50, 70), font, 1.5, (0, 0, 255), 3)

        # 물체 수 화면에 표시
        cv2.putText(img, f"Objects_count: {object_count}", (10, 30), font, 1, (0, 255, 0), 1)

        cv2.imshow('Webcam', img)

        if cv2.waitKey(1) == ord('q'):
            # CSV 및 JSON 파일 저장
            with open(os.path.join(output_dir, 'output.csv'), 'w', newline='') as file:
                writer = csv.writer(file)
                writer.writerows(csv_output)
            with open(os.path.join(output_dir, 'output.json'), 'w') as file:
                json.dump(csv_output, file)
            break

    cap.release()
    cv2.destroyAllWindows()

def main(pt_file):
    if os.path.exists(pt_file):
        model = YOLO(pt_file)
        output_dir = './output'

        if os.path.exists(output_dir) and os.path.isdir(output_dir):
            shutil.rmtree(output_dir)

        os.mkdir(output_dir)
        run_yolo(model, output_dir)
    else:
        print(f"Not found: {pt_file}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Please provide a .pt filename as an argument")
    else:
        main(sys.argv[1])
