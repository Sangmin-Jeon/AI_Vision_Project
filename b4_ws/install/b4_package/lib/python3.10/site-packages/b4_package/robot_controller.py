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
class RobotControllerNode(Node):
    def __init__(self, pt_file):
        # 노드 초기화, 노드 이름 'robot_controller'로 설정
        super().__init__('robot_controller')
        self.pt_file = pt_file

        # YOLO 모델 로드, .pt 파일을 사용하여 모델을 초기화
        self.model = YOLO(self.pt_file)
        self.get_logger().info(f'Loaded model from {self.pt_file}')

        # 웹캠 설정, 인덱스 0인 기본 웹캠 장치를 사용하여 영상을 읽음
        self.cap = cv2.VideoCapture('/dev/video0')
        if not self.cap.isOpened():
            self.get_logger().error('Could not open video device')  # 카메라가 열리지 않을 경우 오류 메시지

        # ROS2 퍼블리셔 설정: 탐지된 이미지와 경고 메시지를 발행
        self.image_publisher = self.create_publisher(Image, 'tracing_image', 10)
        self.alert_publisher = self.create_publisher(String, 'alert_message', 10)
        self.bridge = CvBridge()  # OpenCV 이미지를 ROS 이미지로 변환하기 위한 브릿지 설정

    # 경고 메시지를 퍼블리싱하는 함수
    def publish_alert(self, message):
        msg = String()
        msg.data = message
        self.alert_publisher.publish(msg)
        self.get_logger().info(f'Published alert message: {message}')

    # 탐지된 이미지를 퍼블리싱하는 함수
    def publish_img(self, img):
        # OpenCV 이미지를 RGB로 변환
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # RGB 이미지를 ROS Image 메시지로 변환 후 발행
        image_msg = self.bridge.cv2_to_imgmsg(img_rgb, encoding='rgb8')
        self.image_publisher.publish(image_msg)

    # 프레임을 처리하는 함수
    def process_frame(self):
        # 카메라에서 프레임을 읽음
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture frame')
            return

        # YOLO 모델을 사용하여 프레임 분석
        results = self.model(frame, stream=True)
        object_detected = False  # 객체 감지 여부 확인
        classNames = ['car']  # 탐지할 물체 클래스 (여기서는 'car'만 사용)

        # 탐지된 결과 순회
        for r in results:
            for box in r.boxes:
                # 바운딩 박스 좌표 및 정보 추출
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                confidence = math.ceil((box.conf[0] * 100)) / 100
                cls = int(box.cls[0])

                # 탐지된 객체가 'car'일 경우 경고 메시지 및 이미지 퍼블리시
                if classNames[cls] == 'car' and confidence > 0.5:  # 특정 조건으로 필터링
                    object_detected = True
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    cv2.putText(frame, f'{classNames[cls]}: {confidence}', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                    # 경고 메시지 발행
                    self.publish_alert("ALERT: Vehicle detected!")

        if object_detected:
            # 이미지 퍼블리시
            self.publish_img(frame)

        # 화면에 결과 프레임 출력
        cv2.imshow('Invader Detection', frame)

        # 'q' 키를 눌러 프로그램 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()

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
    node = RobotControllerNode(pt_file)

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
