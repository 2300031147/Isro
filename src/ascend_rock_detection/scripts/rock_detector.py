#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import json
import numpy as np
from ultralytics import YOLO

class RockDetectorNode(Node):
    def __init__(self):
        super().__init__('rock_detector_node')
        
        # Parameters
        self.declare_parameter('model_path', 'yolov8n.pt') # Default to nano model
        self.declare_parameter('confidence_threshold', 0.5)
        
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.conf_thres = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        
        # Initialize YOLO
        self.get_logger().info(f"Loading YOLO model from {self.model_path}...")
        try:
            self.model = YOLO(self.model_path)
            self.get_logger().info("Model loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            self.model = None

        self.bridge = CvBridge()
        
        # Subscribers
        self.img_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_cb,
            10
        )
        
        # Publishers
        self.detection_pub = self.create_publisher(String, '/rock_detections', 10)
        # Optional: Publish annotated image for dashboard
        self.annotated_pub = self.create_publisher(CompressedImage, '/rock_detections/annotated/compressed', 10)

    def image_cb(self, msg):
        if self.model is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return

        # Inference
        results = self.model(cv_image, verbose=False, conf=self.conf_thres)
        
        detections_list = []
        
        for r in results:
            boxes = r.boxes
            for box in boxes:
                cls_id = int(box.cls[0])
                cls_name = self.model.names[cls_id]
                conf = float(box.conf[0])
                
                # Filter for "rock" class if your model has specific classes
                # For demo, we accept all or check specific names
                # if cls_name not in ['rock']: continue

                detections_list.append({
                    "class": cls_name,
                    "confidence": conf,
                    "box": box.xyxy[0].tolist()
                })
                
                # Annotate
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(cv_image, f"{cls_name} {conf:.2f}", (x1, y1-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        if detections_list:
            # Publish JSON
            json_str = json.dumps(detections_list)
            msg_out = String()
            msg_out.data = json_str
            self.detection_pub.publish(msg_out)
            
            # Publish annotated image
            try:
                msg_annotated = self.bridge.cv2_to_compressed_imgmsg(cv_image)
                self.annotated_pub.publish(msg_annotated)
            except Exception as e:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = RockDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
