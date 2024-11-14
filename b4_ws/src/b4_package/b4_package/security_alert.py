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
        # 노드 초기화, 노드 이름 'security_alert'로 설정
        super().__init__('security_alert')
        self.pt_file = pt_file

        # YOLO 모델 로드, .pt 파일을 사용하여 모델을 초기화
        self.model = YOLO(self.pt_file)
        self.get_logger().info(f'Loaded model from {self.pt_file}')

        # 웹캠 설정, 인덱스 0인 기본 웹캠 장치를 사용하여 영상을 읽음
        self.cap = cv2.VideoCapture('/dev/video2')
        if not self.cap.isOpened():
            self.get_logger().error('Could not open video device')  # 카메라가 열리지 않을 경우 오류 메시지

        # 경계선 설정을 위한 변수 초기화
        self.point1, self.point2 = None, None  # 경계선을 설정할 두 점
        self.previous_positions = {}  # 각 물체의 이전 위치를 저장
        self.object_alert_status = {}  # 물체가 경계선을 넘은 경고 상태 저장

        # ROS2 퍼블리셔 설정: 탐지된 이미지와 경고 메시지를 발행
        self.image_publisher = self.create_publisher(Image, 'detected_image', 10)
        self.alert_publisher = self.create_publisher(String, 'alert_message', 10)
        self.bridge = CvBridge()  # OpenCV 이미지를 ROS 이미지로 변환하기 위한 브릿지 설정

        # 경계선 설정을 위해 OpenCV 창을 생성하고 마우스 콜백 함수 등록
        cv2.namedWindow('YOLO Detection')
        cv2.setMouseCallback('YOLO Detection', self.draw_line)

        self.frame_sent = False  # 프레임 전송 여부 확인

    # 경계선 설정 함수 (마우스 클릭 이벤트 처리)
    def draw_line(self, event, x, y, flags, param):
        # 왼쪽 버튼 클릭 시, 두 점을 설정하여 경계선을 그림
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.point1 is None:
                self.point1 = (x, y)  # 첫 번째 점 설정
            else:
                self.point2 = (x, y)  # 두 번째 점 설정

    # 물체가 경계선을 넘었는지 확인하는 함수
    def is_crossing_boundary(self, prev_center, curr_center):
        # 경계선이 설정되지 않은 경우 아무것도 하지 않음
        if self.point1 is None or self.point2 is None:
            return False

        # 선의 방향을 계산하는 내적 함수 정의
        def side(point, p1, p2):
            return (p2[0] - p1[0]) * (point[1] - p1[1]) - (p2[1] - p1[1]) * (point[0] - p1[0])

        # 이전 위치와 현재 위치의 방향을 계산해 경계선 교차 여부 확인
        prev_side = side(prev_center, self.point1, self.point2)
        curr_side = side(curr_center, self.point1, self.point2)
        return prev_side > 0 and curr_side <= 0  # 경계선의 외부에서 내부로 이동한 경우 True 반환

    # 경고 메시지를 퍼블리싱하는 함수
    def publish_alert(self, img, message):
        # OpenCV 이미지를 RGB로 변환
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # RGB 이미지를 ROS Image 메시지로 변환 후 발행
        image_msg = self.bridge.cv2_to_imgmsg(img_rgb, encoding='rgb8')
        self.image_publisher.publish(image_msg)

        # Alert 메시지 발행
        alert_msg = String()
        alert_msg.data = message
        self.alert_publisher.publish(alert_msg)

    # 프레임을 처리하는 함수
    def process_frame(self):
        if self.frame_sent:
            return

        # 카메라에서 프레임을 읽음
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture frame')
            return

        # 경계선 그리기
        if self.point1 and self.point2:
            cv2.line(frame, self.point1, self.point2, (0, 0, 255), 2)  # 경계선을 빨간색으로 그림

        # YOLO 모델로 예측 수행
        results = self.model(frame, stream=True)
        object_count = 0  # 탐지된 물체의 수 초기화
        classNames = ['car']  # 탐지할 물체 클래스 (여기서는 'car'만 사용)
        csv_output = []  # 결과를 저장할 CSV 데이터
        confidences = []  # 탐지된 물체의 확률 저장

        # 탐지된 결과 순회
        for r in results:
            for i, box in enumerate(r.boxes):
                # 바운딩 박스 좌표 및 정보 추출
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                confidence = math.ceil((box.conf[0] * 100)) / 100
                cls = int(box.cls[0])
                confidences.append(confidence)

                # 바운딩 박스 그리기 및 텍스트 표시
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(frame, f'{classNames[cls]}: {confidence}', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                # 물체의 중심점 계산 및 표시
                center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2
                cv2.circle(frame, (center_x, center_y), 5, (0, 255, 255), -1)  # 중심점 표시

                # 이전 위치 저장 및 경계선 넘는지 확인
                object_id = f"obj_{i}"
                current_center = (center_x, center_y)

                # 이전 위치가 저장되어 있는 경우, 경계선을 넘었는지 확인
                if object_id in self.previous_positions:
                    if self.is_crossing_boundary(self.previous_positions[object_id], current_center):
                        # 경고 상태 업데이트
                        self.object_alert_status[object_id] = True
                        self.get_logger().warn('ALERT: Vehicle entering!')

                # 현재 위치 저장
                self.previous_positions[object_id] = current_center
                csv_output.append([x1, y1, x2, y2, confidence, cls])
                object_count += 1  # 탐지된 물체 수 증가

        # 경고 메시지 지속적으로 출력 (한 번만 실행)
        for obj_id, alert in self.object_alert_status.items():
            if alert and not self.frame_sent:
                # 경고 메시지 화면에 표시 및 ROS2 메시지 발행
                cv2.putText(frame, "ALERT: Vehicle entering!", (50, 70), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)
                self.publish_alert(frame, "ALERT: Vehicle entering!")
                self.frame_sent = True  # 프레임을 한 번만 전송하도록 설정

        # 물체 수 화면에 표시
        cv2.putText(frame, f"Objects_count: {object_count}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 1)

        # 화면에 결과 프레임 출력
        cv2.imshow('YOLO Detection', frame)

        # 'q' 키를 눌러 프로그램 종료 시 데이터 저장 및 자원 해제
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
            self.cap.release()  # 카메라 장치 해제
            cv2.destroyAllWindows()  # OpenCV 창 닫기

    # 노드 종료 시 자원 해제
    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

# 메인 함수
def main(args=None):
    # ROS2 초기화
    rclpy.init(args=args)
    
    # .pt 파일 인자 확인
    if len(sys.argv) < 2:
        print("Error: No model (.pt) file provided.")
        return
    
    # YOLO 모델 파일 경로를 가져와서 노드 생성
    pt_file = sys.argv[1]
    node = SecurityAlertNode(pt_file)

    try:
        # 노드가 실행 중일 때 계속 프레임을 처리
        while rclpy.ok():
            node.process_frame()
    finally:
        # 노드 종료 및 rclpy 종료
        node.destroy_node()
        rclpy.shutdown()

# 프로그램 시작점
if __name__ == '__main__':
    main()