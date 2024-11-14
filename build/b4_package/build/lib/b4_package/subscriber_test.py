import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

class AlertSubscriber(Node):
    def __init__(self):
        super().__init__('alert_subscriber')
        self.bridge = CvBridge()

        # alert_message 토픽 구독
        self.create_subscription(String, 'alert_message', self.alert_callback, 10)

        # detected_image 토픽 구독 (self.image_subscription으로 인스턴스 변수로 설정)
        self.image_subscription = self.create_subscription(Image, 'detected_image', self.image_callback, 10)

    def alert_callback(self, msg):
        # 경고 메시지 출력
        self.get_logger().info(f'Received alert message: {msg.data}')

    def image_callback(self, msg):
        # ROS2 이미지 메시지를 OpenCV 이미지로 변환
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.get_logger().info(f'Received image {cv_image}')

        # 수신한 이미지 화면에 표시
        cv2.imshow("Received Image", cv_image)
        cv2.waitKey(1)  # 화면을 짧게 표시 후 업데이트


        # 구독 해제하여 추가적인 호출 방지
        self.destroy_subscription(self.image_subscription)


def main(args=None):
    rclpy.init(args=args)
    node = AlertSubscriber()
    
    try:
        rclpy.spin(node)  # 노드가 계속 실행되도록 유지
    except KeyboardInterrupt:
        pass  # Ctrl+C로 종료할 때 처리
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
