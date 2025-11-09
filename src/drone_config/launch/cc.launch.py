from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


# AprilTag config
APRILTAG_PKG = 'apriltag_ros'
CAMERA_NAME = 'camera' 

# Platform config 
PLATFORM = 'CC'


def generate_launch_description():
    
    # 1. Start MAVROS2 bridge
    mavros = Node(
        package="mavros",
        executable="mavros_node",
        parameters=[
            {
                "fcu_url": "serial:///dev/ttyAMA0:921600", 
                "gcs_url": "",
                "tgt_system": 1,
                "tgt_component": 1,
                "plugin_denylist": ["ftp", "distance_sensor"], 
            }
        ],
        arguments=["--log-level", "error"], 
        output="screen",
    )

    # 2. UAV software nodes (Keep your existing nodes)
    uav_nodes = [
        Node(package="fc_interface", executable="fc_interface_node", output="screen"),
        Node(package="state_machine", executable="state_machine_node", output="screen"),
        Node(package="status_monitoring", executable="status_monitoring_node", output="screen"),
        Node(package="trajectory_planner", executable="navigator_node", output="screen"),
        Node(package="trajectory_planner", executable="global_trajectory_planner_node.py", output="screen"),
        Node(package="mission_manager", executable="mission_manager_node", output="screen"),
        Node(package="hardware_interfaces", executable="camera_calibration_node", output="screen", parameters=[{'camera_name': CAMERA_NAME, 'platform': "CC"}]),
        Node(package="hardware_interfaces", executable="camera_capture_node", output="screen", parameters=[{'camera_name': CAMERA_NAME}]),
        Node(package="vision_guidance", executable="apriltag_guidance_node", output="screen"),
    ]

    delayed_uav_nodes = TimerAction(
        period=10.0,
        actions=uav_nodes,
    )

    
    # 3. AprilTag Composable Node Setup 
    apriltag_node = ComposableNode(
        # Use fully qualified plugin name for robustness
        package=APRILTAG_PKG,
        plugin='AprilTagNode', 
        name='apriltag_detector',
        remappings=[    
            ('/image_rect', f'/{CAMERA_NAME}/image_rect'), 
            ('/camera_info', f'/{CAMERA_NAME}/camera_info'),
            ('/detections', f'/{CAMERA_NAME}/detections'),
            ('tag_detections_image', f'/{CAMERA_NAME}/tag_detections_image')
        ],
        parameters=[
            {'tag_family': 'tag36h11'},
            {'tag_size': 0.3},
            {'camera_frame': CAMERA_NAME},
            {'publish_tf': True}
        ]
    )

    # 4. Define the Composable Node Container
    apriltag_container = ComposableNodeContainer(
        # This defines the CONTAINER process, which is the component manager
        name='apriltag_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            apriltag_node,
        ],
        output='screen',
    )


    # ROSbridge WebSocket server
    ros_bridge = Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{'port': 9091}],
    )

    # Web Video Server for streaming image topics
    web_server = Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            output='screen',
            parameters=[{
                'port': 8081,  
                'ros__parameters': {}
            }]
    )

    
 

    return LaunchDescription([
        
        mavros,
        
        TimerAction(period=4.0, actions=[apriltag_container]),

        TimerAction(period=6.0, actions=[ros_bridge, web_server]),
        
        delayed_uav_nodes,
    ])
