from flask import Flask, render_template, redirect, url_for, request, session, Response
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
import mysql.connector

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
        self.isTracing = False
        self.image_subscriber = self.create_subscription(
            Image,
            'detected_image',
            self.image_callback,
            10
        )

        self.tracing_image_subscriber = self.create_subscription(
            Image,
            'tracing_image',
            self.tracing_image_callback,
            10
        )

    def image_callback(self, msg):
        print("이미지를 받았습니다!")
        try:
            # ROS 이미지를 OpenCV 이미지로 변환 (BGR8 형식)
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            print("이미지 변환 성공!")

            # 경고 메시지가 있는 경우 데이터베이스에 저장
            save_alert_to_db(alert_message="Vehicle entering detected", image=self.latest_image)

        except Exception as e:
            self.get_logger().error(f"이미지 변환 에러: {e}")

    def tracing_image_callback(self, msg):
        print("tracing 이미지를 받았습니다!")
        try:
            # ROS 이미지를 OpenCV 이미지로 변환 (BGR8 형식)
            self.latest_tracing_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.isTracing = True
            print("tracing 이미지 변환 성공!")
        except Exception as e:
            self.get_logger().error(f"tracing 이미지 변환 에러: {e}")

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

# Flask 비디오 스트림 생성
def generate(node):
    while rclpy.ok():
        if node.latest_image is not None:
            if node.isTracing:
                _, jpeg = cv2.imencode('.jpg', node.latest_tracing_image)
            else:
                _, jpeg = cv2.imencode('.jpg', node.latest_image)
                
            frame = jpeg.tobytes()
            yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')
            

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

    # 이미지가 있을 때만 템플릿에 전달
    return render_template('index.html')

# Flask 비디오 스트리밍 라우트
@app.route('/video_feed')
def video_feed():
    return Response(generate(node), mimetype='multipart/x-mixed-replace; boundary=frame')

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
    # ROS2 노드를 백그라운드에서 실행
    rclpy.init()
    node = ImageReceiverNode()  # node 인스턴스를 한 번만 생성
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,))
    ros_thread.daemon = True
    ros_thread.start()

    # Flask 서버 실행
    app.run(host='0.0.0.0', port=5001)