from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from datetime import datetime
import os

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

    # Call set_stream_rate service after 5 seconds
    set_stream_rate_cmd = TimerAction(
        period=10.0,  # wait for MAVROS to start
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'service', 'call', '/mavros/set_stream_rate',
                    'mavros_msgs/srv/StreamRate',
                    "{stream_id: 0, message_rate: 50, on_off: true}"
                ],
                output='screen'
            )
        ]
    )

    # 2. UAV software nodes (Keep your existing nodes)
    uav_nodes = [
        Node(package="fc_interface", executable="fc_interface_node", output="screen"),
        Node(package="state_machine", executable="state_machine_node", output="screen"),
        Node(package="status_monitoring", executable="status_monitoring_node", output="screen"),
        Node(package="navigation", executable="navigator_node", output="screen"),
        Node(package="navigation", executable="global_trajectory_planner_node.py", output="screen"),
        Node(package="mission_manager", executable="mission_manager_node", output="screen"),
        Node(package="hardware_interfaces", executable="camera_calibration_node", output="screen", parameters=[{'camera_name': CAMERA_NAME, 'platform': "CC"}]),
        Node(package="hardware_interfaces", executable="camera_capture_node", output="screen", parameters=[{'camera_name': CAMERA_NAME}]),
	Node(package="hardware_interfaces", executable="video_logger_node", output="screen", parameters=[{'camera_name': CAMERA_NAME}]),
        Node(package="vision_guidance", executable="apriltag_guidance_node", output="screen"),
    ]


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


    # 5. ROSbridge WebSocket server
    ros_bridge = Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{'port': 9091}],
    )

    # 6. Web Video Server for streaming image topics
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

    # 7. Bag logger
    now = datetime.now()
    filename = now.strftime("%Y_%m_%d_%H_%M_%S_")

    bag_folder = os.path.join("data/bag/", f"bag_{filename}")

    bag_logger = ExecuteProcess(
    cmd=[
        'ros2', 'bag', 'record',
        '-o', bag_folder,
        '/drone/fc_interface/state',
        '/drone/fc_interface/battery',
        '/drone/fc_interface/fix',
        "/drone/fc_interface/imu",
        "/drone/fc_interface/mag",
        "/drone/fc_interface/global_origin",
        "/drone/fc_interface/position_local",
        "/drone/fc_interface/position_global",
        "/drone/fc_interface/velocity_local",
        "/drone/fc_interface/heading",
        "/drone/fc_interface/rc",
        '/drone/camera_calibration/detector_status', 
        '/drone/mission_manager/mission',
        "/drone/navigator/setpoint_local",
        '/drone/apriltag_guidance/apriltag_distance',
        '/drone/status_monitoring/prearm',
        '/drone/status_monitoring/components_status',
        '/drone/state_machine/state_machine',
        
    ],
    output='screen'
    )

    
 

    return LaunchDescription([
        
        mavros,

	    set_stream_rate_cmd,
        
        TimerAction(period=0.0, actions=[apriltag_container]),

        TimerAction(period=5.0, actions=[ros_bridge, web_server]),
        
        TimerAction(period=5.0, actions=uav_nodes),

        TimerAction(period=6.0, actions=[bag_logger]),
    ])
