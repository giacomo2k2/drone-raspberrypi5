from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        #1. Start ArduPilot SITL
        ExecuteProcess(
            cmd=[
                "sim_vehicle.py", "-v", "ArduCopter", "-f", "quad",
                "--console", "--map", "--out=127.0.0.1:14550"
            ],
            cwd="/home/giacomo/ardupilot/ArduCopter",  # <-- update this path if needed
            output="screen"
        ),

        #2. Start MAVROS2 bridge
        
        TimerAction(
            period=0.0,
            actions=[
                Node(
                    package="mavros",
                    executable="mavros_node",
                    parameters=[{
                        "fcu_url": "udp://:14550@127.0.0.1:14550",
                        "gcs_url": "",
                        "tgt_system": 1,
                        "tgt_component": 1,
                    }],
                    arguments=["--ros-args", "--log-level", "warn"],
                    output="screen",
                )
            ]
        ),
    
            
        

        # 3. Your nodes
        
        TimerAction(
            period=0.0,
            actions=[
                Node(package="fc_interface", executable="fc_interface_node", output="screen"),
                Node(package="state_machine", executable="state_machine_node", output="screen"),
                Node(package="status_monitoring", executable="status_monitoring_node", output="screen"),
                Node(package="global_trajectory_planner", executable="global_trajectory_planner_node", output="screen"), 
            ]
        ),
            
    ])
