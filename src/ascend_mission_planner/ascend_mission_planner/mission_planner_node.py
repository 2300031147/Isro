#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
import math
from enum import Enum

class MissionState(Enum):
    INIT = 0
    TAKEOFF = 1
    SEARCH = 2
    INSPECT = 3
    RETURN = 4
    LAND = 5
    DONE = 6

class MissionPlannerNode(Node):
    def __init__(self):
        super().__init__('mission_planner_node')
        
        self.state = MissionState.INIT
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.last_detection_time = None
        self.waypoints = []
        self.current_wp_index = 0

        # Subscribers
        self.create_subscription(State, 'mavros/state', self.state_cb, 10)
        self.create_subscription(PoseStamped, 'mavros/local_position/pose', self.pos_cb, 10)
        self.create_subscription(String, '/rock_detections', self.detection_cb, 10)

        # Publishers
        self.target_pos_pub = self.create_publisher(PoseStamped, 'mavros/setpoint_position/local', 10)

        # Clients
        self.arming_client = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, 'mavros/cmd/takeoff')

        self.generate_waypoints()
        
        # Timer (10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Mission Planner (Python) Initialized")

    def state_cb(self, msg):
        self.current_state = msg

    def pos_cb(self, msg):
        self.current_pose = msg

    def detection_cb(self, msg):
        if self.state == MissionState.SEARCH:
            self.get_logger().info("Rock Detected! Pausing for inspection.")
            self.last_detection_time = self.get_clock().now()
            self.state = MissionState.INSPECT

    def generate_waypoints(self):
        # 3x3m Creeping Line
        self.waypoints = [
            {'x': 0.0, 'y': 0.0, 'z': 2.0},
            {'x': 3.0, 'y': 0.0, 'z': 2.0},
            {'x': 3.0, 'y': 1.0, 'z': 2.0},
            {'x': 0.0, 'y': 1.0, 'z': 2.0}
        ]

    def dist(self, p1, p2):
        return math.sqrt((p1.x - p2['x'])**2 + (p1.y - p2['y'])**2)

    def control_loop(self):
        if self.state == MissionState.INIT:
            if self.current_state.connected:
                # Keep publishing setpoint to allow generic offboard mode
                self.publish_setpoint(
                    self.current_pose.pose.position.x,
                    self.current_pose.pose.position.y,
                    self.current_pose.pose.position.z
                )
                if self.current_state.armed and self.current_state.mode == "GUIDED":
                    self.state = MissionState.TAKEOFF
                    self.get_logger().info("Taking off...")
                    self.takeoff()

        elif self.state == MissionState.TAKEOFF:
            self.publish_setpoint(0.0, 0.0, 2.0)
            if self.current_pose.pose.position.z > 1.8:
                self.get_logger().info("Takeoff Complete. Starting Search.")
                self.state = MissionState.SEARCH

        elif self.state == MissionState.SEARCH:
            if self.current_wp_index < len(self.waypoints):
                target = self.waypoints[self.current_wp_index]
                self.publish_setpoint(target['x'], target['y'], target['z'])
                
                d = self.dist(self.current_pose.pose.position, target)
                if d < 0.2:
                    self.get_logger().info(f"Reached WP {self.current_wp_index}")
                    self.current_wp_index += 1
            else:
                self.get_logger().info("Search Complete. Returning Home.")
                self.state = MissionState.RETURN

        elif self.state == MissionState.INSPECT:
            self.publish_setpoint(
                self.current_pose.pose.position.x,
                self.current_pose.pose.position.y,
                2.0
            )
            # Wait 3s
            if (self.get_clock().now() - self.last_detection_time).nanoseconds > 3e9:
                self.get_logger().info("Inspection done.")
                self.state = MissionState.SEARCH

        elif self.state == MissionState.RETURN:
            self.publish_setpoint(0.0, 0.0, 2.0)
            d = math.sqrt(self.current_pose.pose.position.x**2 + self.current_pose.pose.position.y**2)
            if d < 0.2:
                self.state = MissionState.LAND
                self.get_logger().info("Landing...")
                self.land()

        elif self.state == MissionState.LAND:
            if not self.current_state.armed:
                self.state = MissionState.DONE

    def publish_setpoint(self, x, y, z):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        self.target_pos_pub.publish(msg)

    def takeoff(self):
        req = CommandTOL.Request()
        req.altitude = 2.0
        self.takeoff_client.call_async(req)

    def land(self):
        req = SetMode.Request()
        req.custom_mode = "LAND"
        self.set_mode_client.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = MissionPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
