# Autonomous Drone: ROS 2 Companion Computer

ROS 2 (Jazzy) software for the companion computer of an autonomous quadcopter. It runs on a **Raspberry Pi 5** and talks to an **ArduPilot** flight controller (Mamba F405) through MAVROS. It handles mission execution, smooth trajectory generation, health monitoring and **vision-based precision landing on AprilTags**, without needing GPS for the final approach.

Developed as v2 of the drone platform at [NOE APS](https://github.com/NOE-APS), a non-profit in Bologna.

## Architecture

```
 Web GCS ──(rosbridge WebSocket / web_video_server)──┐
                                                     ▼
 ┌──────────────────────── Raspberry Pi 5 · ROS 2 ─────────────────────────┐
 │ mission_manager ─► state_machine ─► navigation (planner + navigator)    │
 │                          ▲                     │                        │
 │ status_monitoring ───────┘                     ▼                        │
 │ camera_ros ─► image_proc ─► apriltag_ros ─► visual_servoing ─► fc_interface
 └──────────────────────────────────────────────────────────────┬──────────┘
                                                     MAVROS / MAVLink (UART)
                                                                ▼
                                              ArduPilot flight controller
```

| Package | Lang | Role |
|---|---|---|
| `fc_interface` | C++ | Bridge to the flight controller over MAVROS: republishes FC state and sensors (IMU, GPS, mag, battery, RC, local/global pose) and forwards setpoints |
| `state_machine` | Python | High-level flight states (idle, pre-arm checks, takeoff, land); exposes arm, disarm, takeoff and land services and drives MAVROS arming and mode changes |
| `mission_manager` | Python | Mission sequencing; converts geodetic waypoints to a local ENU frame |
| `navigation` | C++ / Python | **Global trajectory planner** (minimum-derivative piecewise polynomials via QP, time-scaled to velocity and acceleration limits) and a **navigator** that follows the trajectory as a ROS 2 action, streaming MAVROS setpoints |
| `visual_servoing` | C++ | AprilTag guidance: tag pose → body/ENU frame (tf2, Eigen) → landing setpoints with a cone-shaped descent |
| `camera` | C++ / Python | Camera calibration, detection overlay and logging (built on `camera_ros` + `image_proc`) |
| `status_monitoring` | C++ / Python | GPS, IMU, magnetometer, power, estimator, companion and link health monitors that report to the state machine |
| `drone_interfaces` | — | Custom messages, services and actions (mission, arm/takeoff/land, trajectory following, component status, failsafe events) |
| `drone_config` | Python | Launch files (real hardware and SITL), camera config, safe shutdown |
| `apriltag_ros` | C++ | Git submodule ([christianrauch/apriltag_ros](https://github.com/christianrauch/apriltag_ros)) |

Camera, rectification and AprilTag detection run as **composable nodes** in one multithreaded container to keep image transport zero-copy. A web ground control station connects through `rosbridge_websocket` and `web_video_server`.

## Getting started

Requirements: Ubuntu 24.04, ROS 2 Jazzy, and MAVROS with its GeographicLib datasets.

```bash
mkdir -p ~/drone_ws && cd ~/drone_ws
git clone --recursive https://github.com/giacomo2k2/drone-raspberrypi5.git src
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

**Simulation (ArduPilot SITL):**

```bash
ros2 launch drone_config sitl.launch.py     # MAVROS on udp://:14551@127.0.0.1:14552
```

**On the drone (Raspberry Pi 5, FC on `/dev/ttyAMA0`):**

```bash
ros2 launch drone_config cc.launch.py
```

[`documents/`](documents) covers Pi setup, the ArduPilot custom build and parameters, running the stack as a systemd service, and the Pi ↔ GCS communication design.

## License

[Apache-2.0](LICENSE)
