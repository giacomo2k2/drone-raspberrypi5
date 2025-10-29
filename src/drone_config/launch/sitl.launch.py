from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os

# Get the system's path separator (usually ':')
PATH_SEP = os.pathsep

# --- Ardupilot config ---
ARDUPILOT_PATH = "/media/giacomo/SSD2/Projects/ardupilot/ardupilot"
ARDUPILOT_SITL_SCRIPT = os.path.join(ARDUPILOT_PATH, "Tools/autotest/sim_vehicle.py")
ARDUPILOT_CWD = os.path.join(ARDUPILOT_PATH, "ArduCopter")

# --- Gazebo config ---
GAZEBO_WS_PATH = "/media/giacomo/SSD2/Projects/gz_ws/src/ardupilot_gazebo"
GAZEBO_WORLD = os.path.join(GAZEBO_WS_PATH, "worlds/iris_runway.sdf")
GAZEBO_BIN = "gz" 

# --- AprilTag Configuration ---
APRILTAG_PKG = 'apriltag_ros'
CAMERA_NAME = 'gimbal_camera' 

# --- Platform config ---
PLATFORM = 'SITL'

# -------------------------------

def generate_launch_description():
    # Determine the directory of the launch file to find the config file
    launch_dir = os.path.dirname(os.path.realpath(__file__))
    
    # --- Robust Environment Setup ---
    # ... (set_plugin_path and set_resource_path remain the same) ...
    
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
        Node(package="trajectory_planner", executable="navigator_node", output="screen"),
        Node(package="trajectory_planner", executable="global_trajectory_planner_node.py", output="screen"),
        Node(package="mission_manager", executable="mission_manager_node", output="screen"),
        Node(package="hardware_interfaces", executable="camera_node", output="screen"),
        Node(package="hardware_interfaces", executable="camera_calibration_node", output="screen"),
        Node(package="vision_guidance", executable="apriltag_guidance_node", output="screen"),
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
            (GIMBAL_INFO_TOPIC_GZ, f'/{CAMERA_NAME}/camera_info_gimbal'),
        ],
        output='screen'
    )
    
    
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

    # 9. Define the Composable Node Container (CORRECTED)
    apriltag_container = ComposableNodeContainer(
        # This defines the CONTAINER process, which is the component manager
        name='apriltag_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            apriltag_node, # The COMPOSABLE NODE is passed here
        ],
        output='screen',
    )
    
    delayed_uav_nodes = TimerAction(
        period=12.0, 
        actions=uav_nodes,
    )

    # --- FINAL LAUNCH DESCRIPTION ---
    return LaunchDescription([
        # CRITICAL: These MUST come before the Gazebo process!
        set_plugin_path,         
        set_resource_path,       
        
        # Start Gazebo and Bridge
        gazebo, 
        camera_bridge,
        
        # Give Gazebo time to load (6s) then start ArduPilot SITL
        TimerAction(period=6.0, actions=[ardupilot]), 
        
        # MAVROS (8s)
        TimerAction(period=8.0, actions=[mavros]), 
        
        # Start AprilTag Detector (needs the image bridge and CameraInfo node running)
        TimerAction(period=10.0, actions=[apriltag_container]),
        
        # Finally, start your custom nodes (12s)
        delayed_uav_nodes,
    ])