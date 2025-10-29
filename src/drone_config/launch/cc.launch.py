from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


# --- AprilTag Configuration ---
APRILTAG_PKG = 'apriltag_ros'
CAMERA_NAME = 'gimbal_camera' 


# --- Platform config ---
PLATFORM = 'SITL'
# --------------------------

def generate_launch_description():
    
    # 5. Start MAVROS2 bridge
    mavros = Node(
        package="mavros",
        executable="mavros_node",
        parameters=[
            {
                "fcu_url": "udp://:14551@127.0.0.1:14552", 
                "gcs_url": "",
                "tgt_system": 1,
                "tgt_component": 1,
                "plugin_denylist": ["ftp", "distance_sensor"], 
            }
        ],
        arguments=["--log-level", "error"], 
        output="screen",
    )

    # 6. UAV software nodes (Keep your existing nodes)
    uav_nodes = [
        Node(package="fc_interface", executable="fc_interface_node", output="screen"),
        Node(package="state_machine", executable="state_machine_node", output="screen"),
        Node(package="status_monitoring", executable="status_monitoring_node", output="screen"),
        Node(package="trajectory_planner", executable="navigator_node", output="screen"),
        Node(package="trajectory_planner", executable="global_trajectory_planner_node.py", output="screen"),
        Node(package="mission_manager", executable="mission_manager_node", output="screen"),
        Node(package="hardware_interfaces", executable="camera_calibration_node", output="screen"),
        Node(package="vision_guidance", executable="apriltag_guidance_node", output="screen"),
    ]


    
    # 8. AprilTag Composable Node Setup 

    
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

    # 9. Define the Composable Node Container
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
    
    delayed_uav_nodes = TimerAction(
        period=4.0,
        actions=uav_nodes,
    )

    # --- FINAL LAUNCH DESCRIPTION ---
    return LaunchDescription([
        
        # MAVROS
        mavros,
        
        # Start AprilTag Detector (needs the image bridge and CameraInfo node running)
        # TimerAction(period=4.0, actions=[apriltag_container]),
        
        # Finally, start your custom nodes
        delayed_uav_nodes,
    ])
