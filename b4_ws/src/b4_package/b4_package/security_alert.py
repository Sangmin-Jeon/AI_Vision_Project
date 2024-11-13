import rclpy
from rclpy.node import Node
import sys
from ultralytics import YOLO
import cv2
import math

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

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture frame')
            return

        # YOLO 모델로 예측
        results = self.model(frame, stream=True)
        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                confidence = math.ceil((box.conf[0] * 100)) / 100
                cls = int(box.cls[0])
                # 바운딩 박스 그리기
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(frame, f'Conf: {confidence}', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        
        cv2.imshow('YOLO Detection', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
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
