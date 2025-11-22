from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os

from datetime import datetime

# Get the system's path separator (usually ':')
PATH_SEP = os.pathsep

# Ardupilot config 
ARDUPILOT_PATH = "/media/giacomo/SSD2/Projects/ardupilot/ardupilot"
ARDUPILOT_SITL_SCRIPT = os.path.join(ARDUPILOT_PATH, "Tools/autotest/sim_vehicle.py")
ARDUPILOT_CWD = os.path.join(ARDUPILOT_PATH, "ArduCopter")

# Gazebo config 
GAZEBO_WS_PATH = "/media/giacomo/SSD2/Projects/gz_ws/src/ardupilot_gazebo"
GAZEBO_WORLD = os.path.join(GAZEBO_WS_PATH, "worlds/iris_runway.sdf")
GAZEBO_BIN = "gz" 

# AprilTag config
APRILTAG_PKG = 'apriltag_ros'
CAMERA_NAME = 'camera' 

# Platform config 
PLATFORM = 'SITL'

# -------------------------------

def generate_launch_description():
    
    # 1. Plugin Path (Required to find libArduPilotPlugin.so)
    plugin_path_value = PATH_SEP.join([
        os.path.join(GAZEBO_WS_PATH, 'build'),
        os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    ]).strip(PATH_SEP)

    set_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH', 
        value=plugin_path_value
    )

    # 2. Resource Path (Required to find models like 'runway' and 'iris_with_gimbal')
    resource_path_value = PATH_SEP.join([
        os.path.join(GAZEBO_WS_PATH, 'models'), 
        os.path.join(GAZEBO_WS_PATH, 'worlds'),
        os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    ]).strip(PATH_SEP)

    set_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', 
        value=resource_path_value
    )
    
    # 3. Start Gazebo Harmonic (gz sim)
    gazebo = ExecuteProcess(
        cmd=["bash", "-c", "env __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia gz sim -v6 -r " + GAZEBO_WORLD],
        output="log",
    )

    # 4. Start ArduPilot SITL 
    ardupilot = ExecuteProcess(
        cmd=[
            ARDUPILOT_SITL_SCRIPT,
            "-v", "ArduCopter", 
            "-f", "gazebo-iris",      
            "--model", "JSON",       
            "--console", "--map", 
            "--out=127.0.0.1:14551",  
        ],
        cwd=ARDUPILOT_CWD,
        output="log", 
    )

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
        Node(package="navigation", executable="navigator_node", output="screen"),
        Node(package="navigation", executable="global_trajectory_planner_node.py", output="screen"),
        Node(package="mission_manager", executable="mission_manager_node", output="screen"),
        Node(package="camera", executable="camera_draw_detections_node", output="screen", parameters=[{'camera_name': CAMERA_NAME}]),
        Node(package="camera", executable="camera_draw_node.py", output="screen", parameters=[{'camera_name': CAMERA_NAME}]),
        Node(package="camera", executable="camera_logger_node", output="screen", parameters=[{'camera_name': CAMERA_NAME}]),
        Node(package="visual_servoing", executable="apriltag_guidance_node", output="screen", parameters=[{'platform': PLATFORM, 'camera_frame': 'iris_with_gimbal/gimbal/pitch_link/camera', 'camera_name': CAMERA_NAME}]),
    ]


    # 7. Add the ROS 2 <-> Gazebo Bridge for the Gimbal Camera (remains the same)
    GIMBAL_CAM_TOPIC_GZ = '/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image'
    GIMBAL_INFO_TOPIC_GZ = '/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/camera_info'

    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            f'{GIMBAL_CAM_TOPIC_GZ}@sensor_msgs/msg/Image[gz.msgs.Image',
            f'{GIMBAL_INFO_TOPIC_GZ}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        remappings=[
            (GIMBAL_CAM_TOPIC_GZ, f'/{CAMERA_NAME}/image_raw'),
            (GIMBAL_INFO_TOPIC_GZ, f'/{CAMERA_NAME}/camera_info'),
        ],
        output='screen'
    )
    
    
    image_proc_node = ComposableNode(
        package='image_proc',
        plugin='image_proc::RectifyNode',
        name='image_proc_node',
        remappings=[
                    ('image', f'/{CAMERA_NAME}/image_raw'),
                    ('camera_info', f'/{CAMERA_NAME}/camera_info'),
                    ('image_rect', f'/{CAMERA_NAME}/image_rect')
                ],
        extra_arguments=[{'use_intra_process_comms': True}]
        
    )

    apriltag_node = ComposableNode(
        package='apriltag_ros',
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
            {'tag_size': 0.2},
            {'camera_frame': CAMERA_NAME},
            {'publish_tf': True},
        ], 
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    apriltag_container = ComposableNodeContainer(
        name='camera_apriltag_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',  # multithreaded for efficiency
        composable_node_descriptions=[
            apriltag_node,
            image_proc_node,
        ],
        output='screen',
    )


    # 10. ROSbridge WebSocket server
    ros_bridge = Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{'port': 9091}],
    )

    # 11. Web Video Server for streaming image topics
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

    # 12. Bag logger
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
    
        set_plugin_path,         
        set_resource_path,       
        
        gazebo, 
        camera_bridge,
        
        TimerAction(period=0.0, actions=[ardupilot]), 
        
        TimerAction(period=2.0, actions=[mavros]), 
                
        TimerAction(period=2.0, actions=[ros_bridge, web_server]),

        TimerAction(period=5.0, actions=uav_nodes),

        TimerAction(period=10.0, actions=[apriltag_container]),

        TimerAction(period=6.0, actions=[bag_logger]),
    ])
