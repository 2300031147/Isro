#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class VioBridgeNode(Node):
    def __init__(self):
        super().__init__('vio_bridge_node')
        
        self.declare_parameter('vio_topic', '/visual_slam/tracking/odometry')
        self.declare_parameter('target_topic', '/mavros/vision_pose/pose')
        
        vio_topic = self.get_parameter('vio_topic').get_parameter_value().string_value
        target_topic = self.get_parameter('target_topic').get_parameter_value().string_value
        
        # Subscriber (VIO Source)
        # Note: Isaac ROS usually uses PoseStamped or Odometry. Adjust msg type if needed.
        self.create_subscription(PoseStamped, vio_topic, self.vio_cb, 10)
        
        # Publisher (MAVROS)
        self.vision_pub = self.create_publisher(PoseStamped, target_topic, 10)
        
        self.get_logger().info(f"VIO Bridge (Python) Started. {vio_topic} -> {target_topic}")

    def vio_cb(self, msg):
        # Create new message to ensure timestamp is fresh
        vision_msg = PoseStamped()
        vision_msg.header.stamp = self.get_clock().now().to_msg()
        vision_msg.header.frame_id = "map" # ENU frame 
        
        # Pass through position/orientation
        # Coordinate transformation (ENU to NED) is handled by MAVROS's vision_pose plugin usually.
        # If manual transform needed:
        # MAVROS usually expects ENU frame if using 'vision_pose/pose'
        vision_msg.pose = msg.pose
        
        self.vision_pub.publish(vision_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VioBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
