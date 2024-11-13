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
import rclpy  # ROS2 파이썬 클라이언트 라이브러리
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge  # OpenCV 이미지를 ROS 메시지로 변환

# ROS2 노드 클래스 정의
class AlertPublisher(Node):
    def __init__(self):
        super().__init__('alert_publisher')
        self.image_publisher = self.create_publisher(Image, 'detected_image', 10)
        self.alert_publisher = self.create_publisher(String, 'alert_message', 10)
        self.bridge = CvBridge()

    def publish_alert(self, img, message):
        # OpenCV 이미지를 ROS Image 메시지로 변환 후 발행
        image_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        self.image_publisher.publish(image_msg)

        # Alert 메시지 발행
        alert_msg = String()
        alert_msg.data = message
        self.alert_publisher.publish(alert_msg)

# 경계선 설정 코드 및 YOLO 실행 코드
point1, point2 = None, None
previous_positions = {}

# 경계선 설정하는 함수
def draw_line(event, x, y, flags, param):
    global point1, point2
    if event == cv2.EVENT_LBUTTONDOWN:
        if point1 is None:
            point1 = (x, y)
        else:
            point2 = (x, y)

def is_crossing_boundary(prev_center, curr_center, p1, p2):
    def side(point, p1, p2):
        return (p2[0] - p1[0]) * (point[1] - p1[1]) - (p2[1] - p1[1]) * (point[0] - p1[0])
    
    prev_side = side(prev_center, p1, p2)
    curr_side = side(curr_center, p1, p2)

    return prev_side > 0 and curr_side <= 0

def run_yolo(model, output_dir, node):
    global img, previous_positions, point1, point2
    cap = cv2.VideoCapture('/dev/video2')  # USB 카메라 설정
    cap.set(3, 640)
    cap.set(4, 480)

    cv2.namedWindow('Webcam')
    cv2.setMouseCallback('Webcam', draw_line)

    alert_triggered = False

    while rclpy.ok():  # ROS2 노드가 실행 중일 때만 루프 실행
        ret, img = cap.read()
        if not ret:
            break

        if point1 and point2:
            cv2.line(img, point1, point2, (0, 0, 255), 2)  # 빨간색 선

        results = model(img, stream=True)
        object_count = 0
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
                cv2.putText(img, f"{classNames[cls]}: {confidence}", org, font, 1, color, thickness)

                center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2
                cv2.circle(img, (center_x, center_y), 5, (0, 255, 255), -1)

                object_id = f"obj_{i}"
                current_center = (center_x, center_y)

                if object_id in previous_positions:
                    if point1 and point2:
                        if is_crossing_boundary(previous_positions[object_id], current_center, point1, point2):
                            alert_triggered = True
                            node.publish_alert(img, "ALERT: Vehicle entering!")  # ROS2 토픽으로 메시지와 이미지 발행

                previous_positions[object_id] = current_center
                csv_output.append([x1, y1, x2, y2, confidence, cls])
                object_count += 1

        if alert_triggered:
            cv2.putText(img, "ALERT: Vehicle entering!", (50, 70), font, 1.5, (0, 0, 255), 3)

        cv2.putText(img, f"Objects_count: {object_count}", (10, 30), font, 1, (0, 255, 0), 1)
        cv2.imshow('Webcam', img)

        if cv2.waitKey(1) == ord('q'):
            with open(os.path.join(output_dir, 'output.csv'), 'w', newline='') as file:
                writer = csv.writer(file)
                writer.writerows(csv_output)
            with open(os.path.join(output_dir, 'output.json'), 'w') as file:
                json.dump(csv_output, file)
            break

    cap.release()
    cv2.destroyAllWindows()

def main(pt_file):
    rclpy.init()
    node = AlertPublisher()

    if os.path.exists(pt_file):
        model = YOLO(pt_file)
        output_dir = './output'

        if os.path.exists(output_dir) and os.path.isdir(output_dir):
            shutil.rmtree(output_dir)

        os.mkdir(output_dir)
        try:
            run_yolo(model, output_dir, node)
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        print(f"Not found: {pt_file}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Please provide a .pt filename as an argument")
    else:
        main(sys.argv[1])
