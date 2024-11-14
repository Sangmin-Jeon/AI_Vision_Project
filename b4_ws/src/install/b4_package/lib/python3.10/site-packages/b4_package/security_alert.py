import json
import csv
import time
import math
import os
import shutil
import sys
import numpy as np
import rclpy  # ROS2 Python client library
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge  # Converts OpenCV images to ROS messages
from ultralytics import YOLO  # YOLO object detection library


# Boundary drawing function
def draw_line(event, x, y, flags, param):
    global point1, point2
    if event == cv2.EVENT_LBUTTONDOWN:
        if point1 is None:
            point1 = (x, y)
        else:
            point2 = (x, y)


def is_crossing_boundary(prev_center, curr_center, p1, p2):
    def side(point, p1, p2):
        return (p2[0] - p1[0]) * (point[1] - p1[1]) - (p2[1] - p1[1]) * (point[0] - p1[0])
    
    prev_side = side(prev_center, p1, p2)
    curr_side = side(curr_center, p1, p2)

    return prev_side > 0 and curr_side <= 0


# ROS2 node class for publishing alerts and images
class AlertPublisher(Node):
    def __init__(self):
        super().__init__('yolo_node')
        self.alert_publisher = self.create_publisher(String, 'vehicle_alert', 10)
        self.image_publisher = self.create_publisher(Image, 'vehicle_image', 10)
        self.bridge = CvBridge()
        self.previous_positions = {}  # Store the previous positions of detected objects


    def publish_alert(self, img, message):
        # Publish the alert message to a ROS topic
        self.alert_publisher.publish(String(data=message))

        # Convert the image from OpenCV to a ROS Image message and publish it
        ros_image = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        self.image_publisher.publish(ros_image)


def run_yolo(model, output_dir, node):
    global img, point1, point2
    cap = cv2.VideoCapture('/dev/video2')  # USB camera input
    cap.set(3, 640)  # Set frame width
    cap.set(4, 480)  # Set frame height

    cv2.namedWindow('Webcam')
    cv2.setMouseCallback('Webcam', draw_line)

    alert_triggered = False

    while rclpy.ok():  # ROS2 node should be running while the loop is executing
        ret, img = cap.read()
        if not ret:
            break

        if point1 and point2:
            cv2.line(img, point1, point2, (0, 0, 255), 2)  # Red line

        results = model(img, stream=True)  # Run YOLO inference
        object_count = 0
        classNames = ['car']
        csv_output = []
        confidences = []

        for r in results:
            boxes = r.boxes
            for i, box in enumerate(boxes):
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                # Draw bounding boxes and confidence
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
                confidence = math.ceil((box.conf[0] * 100)) / 100
                cls = int(box.cls[0])
                confidences.append(confidence)

                org = (x1, y1)
                font = cv2.FONT_HERSHEY_SIMPLEX
                color = (255, 0, 0)
                thickness = 2
                cv2.putText(img, f"{classNames[cls]}: {confidence}", org, font, 1, color, thickness)

                # Calculate center of the bounding box
                center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2
                cv2.circle(img, (center_x, center_y), 5, (0, 255, 255), -1)

                object_id = f"obj_{i}"
                current_center = (center_x, center_y)

                if object_id in node.previous_positions:
                    if point1 and point2:
                        if is_crossing_boundary(node.previous_positions[object_id], current_center, point1, point2):
                            alert_triggered = True
                            node.publish_alert(img, "ALERT: Vehicle entering!")  # Publish alert and image

                node.previous_positions[object_id] = current_center
                csv_output.append([x1, y1, x2, y2, confidence, cls])
                object_count += 1

        if alert_triggered:
            cv2.putText(img, "ALERT: Vehicle entering!", (50, 70), font, 1.5, (0, 0, 255), 3)

        cv2.putText(img, f"Objects_count: {object_count}", (10, 30), font, 1, (0, 255, 0), 1)
        cv2.imshow('Webcam', img)

        if cv2.waitKey(1) == ord('q'):
            with open(os.path.join(output_dir, 'output.csv'), 'w', newline='') as file:
                writer = csv.writer(file)
                writer.writerows(csv_output)
            with open(os.path.join(output_dir, 'output.json'), 'w') as file:
                json.dump(csv_output, file)
            break

    cap.release()
    cv2.destroyAllWindows()


def main(pt_file):
    rclpy.init()

    if os.path.exists(pt_file):
        model = YOLO(pt_file)  # Load the YOLO model
        output_dir = './output'

        if os.path.exists(output_dir) and os.path.isdir(output_dir):
            shutil.rmtree(output_dir)

        os.mkdir(output_dir)

        # Create and initialize the ROS2 node
        node = AlertPublisher()

        try:
            run_yolo(model, output_dir, node)
        finally:
            node.destroy_node()  # Shutdown the ROS2 node
            rclpy.shutdown()
    else:
        print(f"Not found: {pt_file}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Please provide a .pt filename as an argument")
    else:
        main(sys.argv[1])
