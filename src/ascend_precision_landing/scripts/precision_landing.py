#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np

class PrecisionLandingNode(Node):
    def __init__(self):
        super().__init__('precision_landing_node')
        
        # Parameters
        self.declare_parameter('aruco_id', 0)
        self.declare_parameter('marker_size', 0.2) # meters
        
        self.target_id = self.get_parameter('aruco_id').get_parameter_value().integer_value
        self.marker_size = self.get_parameter('marker_size').get_parameter_value().double_value
        
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # Subscribers
        self.img_sub = self.create_subscription(
            Image, '/down_camera/image_raw', self.image_cb, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, '/down_camera/camera_info', self.info_cb, 10)
            
        # Publishers
        self.target_pub = self.create_publisher(
            PoseStamped, '/mavros/landing_target/pose', 10)
            
        self.get_logger().info("Precision Landing Node Started")

    def info_cb(self, msg):
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)
        self.info_sub.destroy() # Only need once

    def image_cb(self, msg):
        if self.camera_matrix is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return
            
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters()
        
        # Detect
        corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        
        if ids is not None and self.target_id in ids:
            index = np.where(ids == self.target_id)[0][0]
            
            # Estimate Pose
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                corners[index], self.marker_size, self.camera_matrix, self.dist_coeffs)
            
            # Publish
            self.publish_target(rvec[0], tvec[0])
            
            # Draw for debug (optional, could publish separate topic)
            aruco.drawDetectedMarkers(frame, corners)
            cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec[0], tvec[0], 0.1)

    def publish_target(self, rvec, tvec):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "down_camera_optical_frame"
        
        msg.pose.position.x = tvec[0]
        msg.pose.position.y = tvec[1]
        msg.pose.position.z = tvec[2]
        
        # Convert rvec to quaternion (simplified, MAVROS might just need position for PL)
        # But we send full pose
        # ... rotation logic omitted for brevity, often internal controller handles alignment
        msg.pose.orientation.w = 1.0 
        
        self.target_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PrecisionLandingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
