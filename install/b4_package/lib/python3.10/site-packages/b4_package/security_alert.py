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
class SecurityAlertNode(Node):
    def __init__(self, pt_file):
        super().__init__('security_alert')
        self.pt_file = pt_file
        # YOLO 모델 로드
        self.model = YOLO(self.pt_file)
        self.get_logger().info(f'Loaded model from {self.pt_file}')
        # 카메라 설정 (웹캠 사용)
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('Could not open video device')
        # 경계선 설정을 위한 변수
        self.point1, self.point2 = None, None
        self.previous_positions = {}
        self.object_alert_status = {}  # 물체 경고 상태 저장
        # ROS2 퍼블리셔 설정
        self.image_publisher = self.create_publisher(Image, 'detected_image', 10)
        self.alert_publisher = self.create_publisher(String, 'alert_message', 10)
        self.bridge = CvBridge()
        # 경계선 설정
        cv2.namedWindow('YOLO Detection')
        cv2.setMouseCallback('YOLO Detection', self.draw_line)

    def draw_line(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.point1 is None:
                self.point1 = (x, y)
            else:
                self.point2 = (x, y)

    def is_crossing_boundary(self, prev_center, curr_center):
        if self.point1 is None or self.point2 is None:
            return False

        def side(point, p1, p2):
            return (p2[0] - p1[0]) * (point[1] - p1[1]) - (p2[1] - p1[1]) * (point[0] - p1[0])

        prev_side = side(prev_center, self.point1, self.point2)
        curr_side = side(curr_center, self.point1, self.point2)
        return prev_side > 0 and curr_side <= 0

    def publish_alert(self, img, message):
        # OpenCV 이미지를 ROS Image 메시지로 변환 후 발행
        image_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        self.image_publisher.publish(image_msg)  # detected_image 토픽으로 퍼블리시

        # Alert 메시지 발행
        alert_msg = String()
        alert_msg.data = message
        self.alert_publisher.publish(alert_msg)  # alert_message 토픽으로 퍼블리시

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture frame')
            return

        # 경계선 그리기
        if self.point1 and self.point2:
            cv2.line(frame, self.point1, self.point2, (0, 0, 255), 2)  # 빨간색 선

        # YOLO 모델로 예측
        results = self.model(frame, stream=True)
        object_count = 0  # 물체 카운트 초기화
        classNames = ['car']
        csv_output = []
        confidences = []

        for r in results:
            for i, box in enumerate(r.boxes):
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                confidence = math.ceil((box.conf[0] * 100)) / 100
                cls = int(box.cls[0])
                confidences.append(confidence)

                # 바운딩 박스 그리기
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(frame, f'{classNames[cls]}: {confidence}', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                # 물체의 중심점 계산
                center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2
                cv2.circle(frame, (center_x, center_y), 5, (0, 255, 255), -1)

                # 이전 위치 저장 및 경계선 넘는지 확인
                object_id = f"obj_{i}"
                current_center = (center_x, center_y)

                if object_id in self.previous_positions:
                    if self.is_crossing_boundary(self.previous_positions[object_id], current_center):
                        # 경고 상태 업데이트
                        self.object_alert_status[object_id] = True
                        self.get_logger().warn('ALERT: Vehicle entering!')

                self.previous_positions[object_id] = current_center
                csv_output.append([x1, y1, x2, y2, confidence, cls])
                object_count += 1

        # 경고 메시지 지속적으로 출력
        for obj_id, alert in self.object_alert_status.items():
            if alert:
                cv2.putText(frame, "ALERT: Vehicle entering!", (50, 70), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)
                self.publish_alert(frame, "ALERT: Vehicle entering!")

        # 물체 수 화면에 표시
        cv2.putText(frame, f"Objects_count: {object_count}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 1)

        cv2.imshow('YOLO Detection', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            # CSV 및 JSON 파일 저장
            output_dir = './output'
            if not os.path.exists(output_dir):
                os.mkdir(output_dir)
            with open(os.path.join(output_dir, 'output.csv'), 'w', newline='') as file:
                writer = csv.writer(file)
                writer.writerows(csv_output)
            with open(os.path.join(output_dir, 'output.json'), 'w') as file:
                json.dump(csv_output, file)
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 2:
        print("Error: No model (.pt) file provided.")
        return
    pt_file = sys.argv[1]  # 인자에서 .pt 파일 경로 가져오기
    node = SecurityAlertNode(pt_file)
    try:
        while rclpy.ok():
            node.process_frame()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

