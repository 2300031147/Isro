from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # RealSense D435i
    # We use the standard realsense2_camera launch file if available
    # For now, we stub it or assume it's installed
    # rs_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
    #     ]),
    #     launch_arguments={'enable_gyro': 'true', 'enable_accel': 'true', 'unite_imu_method': '2'}.items()
    # )
    
    # TF-Luna (I2C) - Usually handled by ArduPilot directly if connected to Cube I2C
    # If connected to Jetson I2C, run a driver:
    tf_luna_node = Node(
        package='tf_luna_driver',
        executable='tf_luna_node',
        name='tf_luna',
        output='screen',
        parameters=[{'serial_port': '/dev/ttyTHS1', 'baud_rate': 115200}]
    )

    # Down-Facing Camera (Arducam/USB) using standard v4l2_camera
    down_cam_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='down_camera',
        output='screen',
        parameters=[{
            'video_device': '/dev/video0', 
            'image_size': [640, 480], 
            'pixel_format': 'YUYV'
        }],
        remappings=[('image_raw', '/down_camera/image_raw')]
    )

    return LaunchDescription([
        # rs_launch, # Uncomment if installed
        # tf_luna_node, # Uncomment if using Jetson I2C
        down_cam_node
    ])
