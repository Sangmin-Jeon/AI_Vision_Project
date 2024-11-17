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
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

app = Flask(__name__)
app.secret_key = 'your_secret_key'  # 세션을 사용하려면 비밀 키가 필요합니다.

# 데이터베이스 연결 설정
DB_CONFIG = {
    'host': 'localhost',
    'user': '1234',
    'password': '1234',
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

        # ActionClient 정의
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # 구독자 설정
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

    def send_goal(self, x, y, z=0.0, orientation_x=0.0, orientation_y=0.0, orientation_z=0.0, orientation_w=1.0):
        goal_msg = NavigateToPose.Goal()

        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = 'map'  # frame_id는 'map'으로 설정
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = z
        goal_msg.pose.pose.orientation.x = orientation_x
        goal_msg.pose.pose.orientation.y = orientation_y
        goal_msg.pose.pose.orientation.z = orientation_z
        goal_msg.pose.pose.orientation.w = orientation_w

        self.get_logger().info(f'Sending goal to robot: x={x}, y={y}, z={z}, orientation=({orientation_x}, {orientation_y}, {orientation_z}, {orientation_w})')
        
        self._action_client.wait_for_server()  # 액션 서버가 준비될 때까지 기다림
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        result = future.result()
        if result.accepted:
            self.get_logger().info("Goal accepted!")
        else:
            self.get_logger().info("Goal rejected!")

    def alert_msg_callback(self, msg):
        self.latest_alert_message = msg.data
        self.get_logger().info(f"Received alert message: {msg.data}")

    def coord_msg_callback(self, msg):
        self.latest_coord_message = {'x': msg.x, 'y': msg.y, 'z': msg.z}
        self.get_logger().info(f"Received coordinates: {self.latest_coord_message}")

    def tracing_image_callback(self, msg):
        try:
            self.latest_tracing_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.isTracing = True
            self.get_logger().info("Received tracing image")
        except Exception as e:
            self.get_logger().error(f"Error converting tracing image: {e}")

    def cctv_video_callback(self, msg):
        try:
            self.latest_cctv_video = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.get_logger().info("Received CCTV video")
        except Exception as e:
            self.get_logger().error(f"Error converting CCTV video: {e}")


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
            time.sleep(0.1)

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
            time.sleep(0.1)

# Flask 라우트 설정
@app.route('/')
def index():
    if 'logged_in' in session and session['logged_in']:
        return redirect(url_for('image_screen'))
    return redirect(url_for('login'))

@app.route('/send_goal', methods=['POST'])
def send_goal():
    # 하드코딩된 좌표값
    x = -1.702646583635383
    y = -0.09896343645916243
    z = 0.0
    orientation_x = 0.0
    orientation_y = 0.0
    orientation_z = 0.09822234723903406
    orientation_w = 0.9951644941932236

    try:
        # ROS2 액션 클라이언트를 통해 목표를 전송
        node.send_goal(x, y, z, orientation_x, orientation_y, orientation_z, orientation_w)
        
        return jsonify({"status": "Moving Goal sent to robot successfully!"}), 200
    except Exception as e:
        return jsonify({"status": f"Error: {str(e)}"}), 500
    
def send_end_goal():
    # 여러 목표 좌표 정의
    goals = [
        {
            'x': 0.24296722586148772,
            'y': -0.03581842329189083,
            'z': 0.0,
            'orientation_x': 0.0,
            'orientation_y': 0.0,
            'orientation_z': 0.19234058671653168,
            'orientation_w': 0.9813282318885667
        },
        {
            'x': 0.27136501536292423,
            'y': -0.5584630186824578,
            'z': 0.0,
            'orientation_x': 0.0,
            'orientation_y': 0.0,
            'orientation_z': -0.09671299989353907,
            'orientation_w': 0.9953123106098871
        },
        {
            'x': -0.10077109740665147,
            'y': -0.10310046161971373,
            'z': 0.0,
            'orientation_x': 0.0,
            'orientation_y': 0.0,
            'orientation_z': -0.04165615445343232,
            'orientation_w': 0.9991320056910157
        }
    ]
    
    # 각 목표를 순차적으로 전송하고 완료 후 다음 목표로 이동
    def send_goal_sequence(goals):
        def goal_response_callback(future, goal):
            result = future.result()
            if result.accepted:
                node.get_logger().info(f"Goal accepted: {goal}")
                # 목표가 완료되면 다음 목표를 전송
                send_next_goal()
            else:
                node.get_logger().info(f"Goal rejected: {goal}")
            

        def send_next_goal():
            if goals:
                goal = goals.pop(0)  # 리스트에서 첫 번째 목표를 가져옴
                node.get_logger().info(f"Sending next goal: {goal}")
                goal_msg = NavigateToPose.Goal()
                goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
                goal_msg.pose.header.frame_id = 'map'  # 'map' frame
                goal_msg.pose.pose.position.x = goal['x']
                goal_msg.pose.pose.position.y = goal['y']
                goal_msg.pose.pose.position.z = goal['z']
                goal_msg.pose.pose.orientation.x = goal['orientation_x']
                goal_msg.pose.pose.orientation.y = goal['orientation_y']
                goal_msg.pose.pose.orientation.z = goal['orientation_z']
                goal_msg.pose.pose.orientation.w = goal['orientation_w']
                
                # 목표를 전송하고 응답을 기다림
                send_goal_future = node._action_client.send_goal_async(goal_msg)
                send_goal_future.add_done_callback(lambda future: goal_response_callback(future, goal))
            else:
                node.get_logger().info("All goals completed")

        send_next_goal()  # 첫 번째 목표를 전송

    # 목표를 순차적으로 전송
    send_goal_sequence(goals)

    return jsonify({"status": "All goals sent to robot successfully!"}), 200



@app.route('/login', methods=['GET', 'POST'])
def login():
    if request.method == 'POST':
        username = request.form['username']
        password = request.form['password']
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
    alert_message = node.latest_alert_message if node.latest_alert_message else "No alert messages"
    coord_message = node.latest_coord_message if node.latest_coord_message else "No Detected Coord"
    return jsonify({'alert_message': alert_message, 'coord_message': coord_message})

@app.route('/video_feed_cctv')
def video_feed_cctv():
    return Response(gen_cctv_video(node), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/video_feed_robot')
def video_feed_robot():
    return Response(gen_robot_camera(node), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/detail_screen')
def detail_screen():
    button_clicked = request.args.get('button', 'None')
    return render_template('detail.html', button=button_clicked)

@app.route('/index_screen', methods=['POST'])
def index_screen():
    return redirect(url_for('image_screen'))

@app.route('/logout')
def logout():
    session.pop('logged_in', None)
    return redirect(url_for('login'))

def ros_spin_thread(node):
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)

if __name__ == '__main__':
    rclpy.init()
    node = ImageReceiverNode()
    ros_thread = threading.Thread(target=ros_spin_thread, args=(node,))
    ros_thread.daemon = True
    ros_thread.start()
    app.run(host='0.0.0.0', port=5001)
