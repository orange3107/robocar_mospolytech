#!/usr/bin/python3
# -*- coding: utf-8 -*-
import cv2
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import time

CAP_WIDTH = 640
CAP_HEIGHT = 480
FPS = 15
VIDEO_DURATION = 25  # продолжительность видео в секундах

class LightsDetectorStream(Node):
    def __init__(self):
        super().__init__('lights_detector_stream')
        self.publisher_ = self.create_publisher(Int32, 'traffic_light_color', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.last_red_time = None
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open video")
            rclpy.shutdown()
            return
        
        self.model_lights = YOLO('/home/polytech/ros2_humble/src/detect_tl/resource/best_traffic_small_yolo.pt')

    def timer_callback(self):
        if self.last_red_time and (time.time() - self.last_red_time) < 10:
            return
        
        ret, frame = self.cap.read()
        if not ret:
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()
            return

        results_lights = self.model_lights(frame)

        frame_with_boxes = frame.copy()
        
        traffic_light_color = None
        
        for result in results_lights:
            boxes = result.boxes
            if boxes is not None:
                for box in boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    conf = box.conf
                    cls = box.cls
                    label = result.names[int(cls)]
                    
                    if 'green' in label.lower():
                        traffic_light_color = 1
                    elif 'yellow' in label.lower():
                        traffic_light_color = 2
                    elif 'red' in label.lower():
                        traffic_light_color = 3
                        self.last_red_time = time.time()
                    elif 'red' in label.lower() and 'yellow' in label.lower():
                        traffic_light_color = 4
                    
                    cv2.rectangle(frame_with_boxes, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame_with_boxes, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36, 255, 12), 2)
        
        if traffic_light_color is not None and (traffic_light_color != 3 or (time.time() - self.last_red_time) >= 10):
            msg = Int32()
            msg.data = traffic_light_color
            self.publisher_.publish(msg)

        cv2.imshow('Frame with Boxes', frame_with_boxes)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    lights_detector_stream = LightsDetectorStream()
    rclpy.spin(lights_detector_stream)
    lights_detector_stream.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
