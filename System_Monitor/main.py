from flask import Flask, render_template, redirect, url_for, request, session, Response, jsonify
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
import mysql.connector
from std_msgs.msg import String
from geometry_msgs.msg import Point
import time


app = Flask(__name__)
app.secret_key = 'your_secret_key'  # 세션을 사용하려면 비밀 키가 필요합니다.

# 데이터베이스 연결 설정
DB_CONFIG = {
    'host': 'localhost',
    'user': '1234',  # 실제 사용자 이름으로 root 사용
    'password': '1234',  # root 사용자의 비밀번호
    'database': 'SecuritySystem'
}

# ROS2 노드 클래스 정의
class ImageReceiverNode(Node):
    def __init__(self):
        super().__init__('flask_image_receiver')
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_tracing_image = None
        self.latest_cctv_video = None
        self.isTracing = False
        self.latest_alert_message = None 
        self.latest_coord_message = None
        
        self.alert_subscriber = self.create_subscription(
            String,
            'alert_message',
            self.alert_msg_callback,
            10
        )

        self.coord_subscriber = self.create_subscription(
            Point,
            'intrusion_coordinates',
            self.coord_msg_callback,
            10
        )

        self.tracing_image_subscriber = self.create_subscription(
            Image,
            'tracing_image',
            self.tracing_image_callback,
            10
        )

        self.cctv_video_subscriber = self.create_subscription(
            Image,
            'cctv_video',
            self.cctv_video_callback,
            10
        )
    

    def alert_msg_callback(self, msg):
        # String 메시지를 수신했을 때 실행되는 콜백 함수
        print("경고 메시지를 받았습니다!")
        
        try:
            # 수신한 메시지 출력
            alert_msg = msg.data
            self.latest_alert_message = alert_msg

            print(f"Alert message: {alert_msg}") 
            # 필요에 따라 추가 처리 수행 (예: 데이터베이스에 저장)
            # save_alert_to_db(alert_message=alert_message)

        except Exception as e:
            self.get_logger().error(f"경고 메시지 처리 중 에러: {e}")

    def coord_msg_callback(self, msg):        
        try:
            # 수신한 메시지 출력
            coord_msg = {'x': msg.x, 'y': msg.y, 'z': msg.z}
            self.latest_coord_message = coord_msg

            print(f"Coord: {coord_msg}") 

            # 필요에 따라 추가 처리 수행 (예: 데이터베이스에 저장)
            # save_alert_to_db(alert_message=alert_message)

        except Exception as e:
            self.get_logger().error(f"좌표 메시지 처리 중 에러: {e}")

    def tracing_image_callback(self, msg):
        print("tracing 이미지를 받았습니다!")
        try:
            # ROS 이미지를 OpenCV 이미지로 변환 (BGR8 형식)
            self.latest_tracing_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.isTracing = True
            print("tracing 이미지 변환 성공!")
        except Exception as e:
            self.get_logger().error(f"tracing 이미지 변환 에러: {e}")


    def cctv_video_callback(self, msg):
        # cctv_video 이미지를 수신하고 화면에 표시
        try:
            self.latest_cctv_video = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # 경고 메시지가 있는 경우 데이터베이스에 저장
            # save_alert_to_db(alert_message="Vehicle entering detected", image=self.cctv_video)
        except Exception as e:
            self.get_logger().error(f"Failed to convert CCTV video: {str(e)}")


# 경고 데이터베이스에 저장하는 함수 정의
def save_alert_to_db(alert_message, image):
    try:
        # 데이터베이스에 연결
        connection = mysql.connector.connect(**DB_CONFIG)
        cursor = connection.cursor()

        # 이미지를 바이너리 데이터로 변환
        _, encoded_image = cv2.imencode('.jpg', image)
        image_binary = encoded_image.tobytes()

        # 데이터베이스에 데이터 삽입
        query = "INSERT INTO alerts (alert_message, image) VALUES (%s, %s)"
        cursor.execute(query, (alert_message, image_binary))
        connection.commit()
        print("데이터베이스에 경고 정보 저장 성공")

    except mysql.connector.Error as err:
        print(f"데이터베이스 에러: {err}")

    finally:
        if connection.is_connected():
            cursor.close()
            connection.close()

def ros_spin_thread(node):
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
            

# Video feed generator function for CCTV camera
def gen_cctv_video(node):
    while True:
        if node.latest_cctv_video is not None:
            resized_frame = cv2.resize(node.latest_cctv_video, (320, 240))
            _, jpeg = cv2.imencode('.jpg', resized_frame)
            frame = jpeg.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')
        else:
            time.sleep(0.1)  # 비동기적으로 대기

# Video feed generator function for robot camera
def gen_robot_camera(node):
    while True:
        if node.latest_tracing_image is not None:
            resized_frame = cv2.resize(node.latest_tracing_image, (320, 240))
            _, jpeg = cv2.imencode('.jpg', resized_frame)
            frame = jpeg.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')
        else:
            time.sleep(0.1)  # 비동기적으로 대기

# Flask 라우트 설정
@app.route('/')
def index():
    # 로그인 상태 확인
    if 'logged_in' in session and session['logged_in']:
        return redirect(url_for('image_screen'))
    return redirect(url_for('login'))

@app.route('/login', methods=['GET', 'POST'])
def login():
    if request.method == 'POST':
        username = request.form['username']
        password = request.form['password']
        
        # 간단한 로그인 검증 (여기서는 사용자와 비밀번호를 하드코딩)
        if username == 'user' and password == 'password':
            session['logged_in'] = True
            return redirect(url_for('image_screen'))
        else:
            return 'Invalid credentials, please try again.'
    
    return render_template('login.html')

@app.route('/image_screen')
def image_screen():
    if 'logged_in' not in session or not session['logged_in']:
        return redirect(url_for('login'))
    
    return render_template('index.html')

@app.route('/get_alert_data')
def get_alert_data():
    alert_message = node.latest_alert_message if node.latest_alert_message is not None else "No alert messages"
    coord_message = node.latest_coord_message if node.latest_coord_message is not None else "No Detected Coord"
    return jsonify({'alert_message': alert_message, 'coord_message': coord_message})

# Route for the video feed of CCTV
@app.route('/video_feed_cctv')
def video_feed_cctv():
    return Response(gen_cctv_video(node), mimetype='multipart/x-mixed-replace; boundary=frame')

# Route for the video feed of robot camera
@app.route('/video_feed_robot')
def video_feed_robot():
    return Response(gen_robot_camera(node), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/detail_screen')
def detail_screen():
    button_clicked = request.args.get('button', 'None')
    return render_template('detail.html', button=button_clicked)

@app.route('/index_screen', methods=['POST'])
def index_screen():
    # 버튼 2 클릭 시 index로 리디렉션
    return redirect(url_for('image_screen'))

@app.route('/logout')
def logout():
    session.pop('logged_in', None)
    return redirect(url_for('login'))

if __name__ == '__main__':
    rclpy.init()
    node = ImageReceiverNode()  # ROS 노드 인스턴스 생성
    ros_thread = threading.Thread(target=ros_spin_thread, args=(node,))
    ros_thread.daemon = True
    ros_thread.start()
    app.run(host='0.0.0.0', port=5001)