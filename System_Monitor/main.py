from flask import Flask, render_template, redirect, url_for, request, session, Response
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading

app = Flask(__name__)
app.secret_key = 'your_secret_key'  # 세션을 사용하려면 비밀 키가 필요합니다.

# ROS2 노드 클래스 정의
class ImageReceiverNode(Node):
    def __init__(self):
        super().__init__('flask_image_receiver')
        self.bridge = CvBridge()
        self.latest_image = None
        self.image_subscriber = self.create_subscription(
            Image,
            'detected_image',
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        print("이미지를 받았습니다!")
        try:
            # ROS 이미지를 OpenCV 이미지로 변환 (BGR8 형식)
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            print("이미지 변환 성공!")
        except Exception as e:
            self.get_logger().error(f"이미지 변환 에러: {e}")

# Flask 비디오 스트림 생성
def generate(node):
    while rclpy.ok():
        if node.latest_image is not None:
            _, jpeg = cv2.imencode('.jpg', node.latest_image)
            print(jpeg)
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