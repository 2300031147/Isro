from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Isaac ROS Visual SLAM Node
    # Assumes 'isaac_ros_visual_slam' package is installed on the Jetson
    visual_slam_node = Node(
        package='isaac_ros_visual_slam',
        executable='isaac_ros_visual_slam',
        name='visual_slam_node',
        output='screen',
        parameters=[{
            'enable_rectified_pose': True,
            'denoise_input_images': False,
            'rectified_images': True,
            'enable_debug_mode': False,
            'd4xx_enable_projector': True,
            # Add other VIO tuning params here
        }],
        remappings=[
            ('stereo_camera/left/image', '/camera/infra1/image_rect_raw'),
            ('stereo_camera/left/camera_info', '/camera/infra1/camera_info'),
            ('stereo_camera/right/image', '/camera/infra2/image_rect_raw'),
            ('stereo_camera/right/camera_info', '/camera/infra2/camera_info'),
        ]
    )

    return LaunchDescription([visual_slam_node])
