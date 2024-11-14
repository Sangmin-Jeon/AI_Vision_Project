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

        # detected_image 토픽 구독
        self.create_subscription(Image, 'detected_image', self.image_callback, 10)

    def alert_callback(self, msg):
        # 경고 메시지 출력
        self.get_logger().info(f'Received alert message: {msg.data}')

    def image_callback(self, msg):
        # ROS2 이미지 메시지를 OpenCV 이미지로 변환
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # 수신한 이미지 화면에 표시
            cv2.imshow("Received Image", cv_image)
            cv2.waitKey(10)  # 적절한 화면 업데이트를 위해 필요 (여기서 waitKey 값을 증가시킴)
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {str(e)}")

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