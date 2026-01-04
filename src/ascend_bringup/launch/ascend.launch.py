import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # 1. MAVROS
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('ascend_bringup'), 'config', 'mavros_config.yaml')]
    )

    # 2. Sensors (RealSense, DownCam, LiDAR)
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ascend_sensors'), 'launch', 'sensors.launch.py')
        ])
    )

    # 3. VIO (Isaac ROS Visual SLAM)
    vio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ascend_vio'), 'launch', 'vio.launch.py')
        ])
    )

    # 4. Dashboard Bridge
    dashboard_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ascend_dashboard'), 'launch', 'ascend_dashboard.launch.py')
        ])
    )

    # 5. Core Logic Nodes (All Python now)
    bridge_node = Node(
        package='ascend_mavros_interface',
        executable='vio_bridge', # Python entry point
        name='vio_to_mavlink',
        output='screen'
    )

    mission_node = Node(
        package='ascend_mission_planner',
        executable='mission_planner', # Python entry point
        name='mission_planner',
        output='screen'
    )

    rock_node = Node(
        package='ascend_rock_detection',
        executable='rock_detector.py', # Python script
        name='rock_detector',
        output='screen'
    )

    landing_node = Node(
        package='ascend_precision_landing',
        executable='precision_landing.py', # Python script
        name='precision_landing',
        output='screen'
    )

    return LaunchDescription([
        mavros_node,
        sensors_launch,
        vio_launch,
        dashboard_launch,
        bridge_node,
        mission_node,
        rock_node,
        landing_node
    ])
