import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix, BatteryState, CompressedImage
from geometry_msgs.msg import PoseStamped
import threading
import json
import base64

class ASCENDInterface(Node):
    def __init__(self, sio):
        super().__init__('ascend_web_interface')
        self.sio = sio  # Socket.IO server instance

        # State storage
        self.drone_state = {
            "mode": "UNKNOWN",
            "armed": False,
            "battery": 0,
            "position": {"x": 0, "y": 0, "z": 0},
            "satellites": 0,
            "altitude": 0.0
        }

        # Subscribers
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_cb, 10
        )
        self.local_pos_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.pos_cb, 10
        )
        self.gps_sub = self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self.gps_cb, 10
        )
        self.battery_sub = self.create_subscription(
            BatteryState, '/mavros/battery', self.battery_cb, 10
        )
        self.image_sub = self.create_subscription(
            CompressedImage, '/camera/color/image_raw/compressed', self.image_cb, 10
        )
        
        # Publishers (for commands from frontend)
        # self.cmd_pub = self.create_publisher(String, '/ascend/command', 10)

        self.get_logger().info("ASCEND Web Interface Node Started")

    def state_cb(self, msg):
        self.drone_state["mode"] = msg.mode
        self.drone_state["armed"] = msg.armed
        self.emit_telemetry()

    def pos_cb(self, msg):
        self.drone_state["position"] = {
            "x": round(msg.pose.position.x, 2),
            "y": round(msg.pose.position.y, 2),
            "z": round(msg.pose.position.z, 2)
        }
        self.drone_state["altitude"] = round(msg.pose.position.z, 2)
        self.emit_telemetry()

    def gps_cb(self, msg):
        # Extract satellite count if available in status or similar (msg.status.service)
        # For typical NavSatFix, 'status' indicates fix type.
        # Often MAVROS populates msg.status.status with fix type.
        pass

    def battery_cb(self, msg):
        self.drone_state["battery"] = round(msg.percentage * 100) if msg.percentage <= 1.0 else round(msg.percentage)
        self.emit_telemetry()

    def image_cb(self, msg):
        # Encode image to base64 and functionality to frontend
        encoded_string = base64.b64encode(msg.data).decode('utf-8')
        self.sio.emit('video_frame', encoded_string)

    def emit_telemetry(self):
        self.sio.emit('telemetry', self.drone_state)

def start_ros_node(sio):
    rclpy.init()
    ascend_node = ASCENDInterface(sio)
    
    # Spin in a separate thread so it doesn't block FastAPI
    spin_thread = threading.Thread(target=rclpy.spin, args=(ascend_node,), daemon=True)
    spin_thread.start()
    
    return ascend_node
