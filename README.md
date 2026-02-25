
# Sowbot (ROS 2 stack)

An open-source, containerised ROS2 Jazzy stack for autonomous agricultural robotics. This repository provides the drivers and orchestration for the Sowbot platform, featuring RTK-GNSS localisation and ESP32-based hardware control.

Development is led by the <a href="https://agroecologylab.org.uk" target="_blank">Agroecology Lab</a> building on the core developed by <a href="https://github.com/zauberzeug/" target="_blank">Zauberzeug</a>.

Reference open hardware stack(s) under development at [Sowbot.co.uk](https://sowbot.co.uk) in addition to orginal [Zauberzeug Field Friend](https://github.com/zauberzeug/)

## Sowbot Roadmap

Branch: `sowbot` | ROS 2 Jazzy | Upstream: [zauberzeug/feldfreund\_devkit\_ros](https://github.com/zauberzeug/feldfreund_devkit_ros)  
Open-source precision seeding and weeding robot — [sowbot.co.uk](https://sowbot.co.uk) | [Agroecology Lab](https://agroecologylab.org.uk)  
**Collaborators welcome.** See [CONTRIBUTING.md](CONTRIBUTING.md). Contact: [sowbot.co.uk](https://sowbot.co.uk)

---
| # | Feature | Description | Status | Phase |
|---|---------|-------------|--------|-------|
| **FOUNDATION** | | | | **2025** |
| F1 | Containerised deployment | Full ROS 2 Jazzy stack managed via Docker and `manage.py`. Live volume mapping to `/workspace`. Build, full-build, and runtime modes. | Done | 2025 |
| F2 | Stable device addressing | `fixusb.py` with Jetson/generic architecture detection, kernel `low_latency` mode, and udev symlink generation. Writes `.env` consumed by all launch files. | Done | 2025 |
| F3 | Teleop dashboard | NiceGUI web cockpit on `:80` for joystick control and GPS health monitoring. | Done | 2025 |
| F4 | ublox DGNSS driver | Modified `ublox_dgnss` providing high-bandwidth UBX binary data. Dual F9P moving-base configuration with dynamic port assignment via `fixusb.py`. | Active | 2025 |
| F5 | Diagnostics TUI | `agbot-diagnostic.py` terminal status view of all hardware topics. Run inside container via `login.sh`. | Done | 2025 |
| **MVP FIELD** | | | | **2026** |
| M1 | AOC platform integration | The first non-CUDA, low-cost open hardware entry in the AOC ecosystem. Demonstrates AOC platform abstraction on affordable ARM hardware accessible to smallholders. | In progress | 2026 |
| M2 | Topological navigation + nav2 | LCAS `topological_navigation` (`aoc_refactor` branch) cloned and building in Dockerfile. Self-contained `navigation2.py` with A\* route planning, explicit state machine, and `row_traversal` / `NavigateToPose` / `goal_align` edge actions. `fake_nav2_server` simulator enables full pipeline testing without hardware. Pending: Nav2 topic remapping shim and Jazzy field validation (Issue #8 gate). | In progress | 2026 |
| M3 | Open-field row-crop scenario | `strawberry_polytunnel.tmap2.yaml` (Riseholme) serves as format reference. Caatinga farm maps (`meu_mapa.pgm`, `novo_mapa_fazenda.pgm`) committed to [caatingarobotics](https://github.com/samuk/caatingarobotics). AOC scenario definition for Caatinga semi-arid row-crop conditions pending tmap2 authoring from real field survey. | In progress | 2026 |
| M4 | RTK-GNSS localisation | Dual u-blox F9P moving-base. NAV-RELPOSNED heading vector fused with wheel odometry in `robot_localization` EKF (`gps_ekf.yaml` authored in caatingarobotics). Pending: topic remapping from `ublox_dgnss` native topics (`ublox_nav_sat_fix_hp`, `ubx_nav_rel_pos_ned`) to EKF inputs (`/gps/fix`, `/imu`). Heading validated at antenna separation >= 0.5 m. | Active | 2026 |
| M5 | Dual-SBC ROS 2 stack | Both Avaota A1 boards (8x Cortex-A55, T527) run ROS 2 Jazzy with `rmw_zenoh_cpp`. Static peer Zenoh session over dedicated 1 GbE crossover. Multicast discovery disabled. Pending: `zenoh_config.json` peer config committed to both repos. | Planned | 2026 |
| M6 | NPU perception (Neo board) | Full `camera_source_node` → `yolo_inference_node` → `infestation_analytics_node` pipeline implemented in [caatingarobotics](https://github.com/samuk/caatingarobotics) `caatinga_vision` package. 1 m grid infestation mapping, pump-rate spray recommendations, session-logged CSV/JSON output. Pending: T527 AIPU model conversion, Zenoh bridge to publish `/aoc/conditions` to Limbic System. 200 ms capture-to-publish SLA. | In progress | 2026 |
| M7 | Sentor safety monitoring | [LCAS Sentor](https://github.com/LCAS/sentor) topic-based diagnostics and hardware-software heartbeat. Triggers automated recovery or emergency motor cut-off on topic silence. | Planned | 2026 |
| M8 | Visual crop-row navigation | `sowbot_row_follow` package committed to [caatingarobotics](https://github.com/samuk/caatingarobotics) `jazzy` branch. ExG vegetation index + Otsu threshold, scan-window line fitting, Cherubini & Chaumette visual servoing. Publishes `/aoc/conditions/row_offset` (Float32), `/aoc/conditions/row_heading_error` (Float32, rad), `/aoc/heartbeat/neo_vision` (Bool), `/caatinga_vision/row_nav/debug_image`. Camera calibration (`camera_height_m`, `camera_tilt_deg`) required before field use. | In progress | 2026 |
| M9 | Vizanti web visualisation | [Vizanti](https://github.com/MoffKalast/vizanti/tree/ros2) web-based mission planner and live telemetry for field operators. | Research | 2026 |
| **PRODUCTION** | | | | **2027** |
| P1 | STM32H7 + copper-rs MCU | Replace ESP32/Lizard DSL with STM32H745 running [copper-rs](https://github.com/copper-project/copper-rs) statically-scheduled Rust firmware. Hard real-time motor PID, hardware safety interlocks. | Research | 2027 |
| P2 | CANopen bus | ISO 11898 FDCAN at 500 kbit/s arbitration / 2 Mbit/s data phase. [lely-core](https://github.com/lely-industries/lely-core) CANopen master on Limbic T527 native M_CAN peripheral. DSP402 drive profile. | Research | 2027 |
| P3 | RT kernel + core isolation | PREEMPT_RT kernel on the Limbic System (Avaota A1 / T527) running LCAS `topological_navigation` (`aoc_refactor`) and Nav2. `isolcpus=4-7` with RTK EKF on core 2 (SCHED_FIFO 60), AOC navigation on cores 4-6, watchdog on core 5. GbE/CAN IRQ affinity pinned to core 0. | Planned | 2027 |
| P4 | ROFS image | Read-only rootfs for the Limbic System. Ubuntu Noble minimal or Yocto with RT kernel, pre-built LCAS `topological_navigation` (AOC branch), Nav2, and `rmw_zenoh_cpp`. Immutable field deployment — no colcon build on boot. | Research | 2027 |
| **END-EFFECTORS** | | | | **TBD** |
| E1 | Delta weeding module | [Open-Weeding-Delta](https://github.com/Agroecology-Lab/Open-Weeding-Delta) precision mechanical weeding end-effector. CANopen actuator node on delta controller. | Research | TBD |
| E2 | LASER weeding module | [Laudando LASER](https://github.com/Laudando-Associates-LLC/LASER) integration and validation on Sowbot. Requires E-Stop interlocking with CANopen safety chain. | Research | TBD |
| **DATASETS & COLLABORATION** | | | | **Ongoing** |
| D1 | UK open-field dataset | Field imagery and GNSS logs from UK agroecological farm conditions. Published under CC licence for training and benchmarking. | Planned | 2026 |
| D2 | Caatinga biome dataset | Semi-arid row-crop imagery from Brazilian Caatinga conditions contributed by [caatingarobotics](https://github.com/joaodemouragy-hash/caatingarobotics). Validated on T527 AIPU. | Active | 2026 |
---

### Collaboration

This project is built on and aims to maintain upstream compatibility with [zauberzeug/feldfreund\_devkit\_ros](https://github.com/zauberzeug/feldfreund_devkit_ros).

High Level navigation is developed from the work of [Lincoln Centre for Autonomous Systems (LCAS)](https://lcas.lincoln.ac.uk) as part of the [Agri-OpenCore](https://agri-opencore.org) open ROS 2 ecosystem for agricultural robotics.

Perception models, Nav2 stack, datasets and simulation environments are developed in collaboration with [caatingarobotics](https://github.com/joaodemouragy-hash/caatingarobotics), The [Sowbot Jazzy fork](https://github.com/samuk/caatingarobotics) is pending upstream merge.

# ⚠️ CRITICAL SAFETY WARNING: 

 This software is under active development and may be broken at any given moment. For a stable reference implementation see the upstream Zauberzeug project.

**THIS SOFTWARE COULD CONTROL PHYSICAL HARDWARE CAPABLE OF PRODUCING SIGNIFICANT KINETIC FORCE.**

1. **EXPERIMENTAL STATUS**: This branch ('sowbot') contains experimental code generated and 
   refined with AI assistance. It has NOT undergone full-scale field validation.
2. **STATUTORY NOTICE (UK)**: Usage of this software is at the user's sole risk. While 
   standard open-source licenses apply, users are reminded that operating agricultural 
   robotics requires a professional duty of care.
3. **MANDATORY HARDWARE SAFETY**: Under no circumstances should this software be used 
   to control a robot of any size without a independent, hard-wired, physical Emergency Stop (E-Stop) 
   system. Software-based stops (such as /estop/soft) are NOT a substitute for 
   Category 0 or 1 hardware safety stops.
4. **NO LIABILITY**: To the extent permitted by the laws of England and Wales, the 
   contributors exclude all liability for property damage, crop loss, or indirect 
   consequential damages.

### Health Warning

This repo may contain traces of LLM slop, We've done our best to mitigate this. If you are allergic to slop, please help us refactor.


## Quick Start

### 1. Clone the Repository
Open a terminal on your host machine and download the workspace:
```bash
git clone -b sowbot https://github.com/Agroecology-Lab/feldfreund_devkit_ros.git
cd feldfreund_devkit_ros
```

### 2. Build & Launch
Use the management script to build the ROS 2 workspace and launch the robot stack. This script automatically handles hardware discovery and port permissions:
```bash
./manage.py full-build
xhost +local:docker
./manage.py 
```
*Note: manage.py automatically detects hardware ports (ESP32 & u-blox), updates your .env configuration, and launches the container with live volume mapping to /workspace.*


## Management & Tools

### manage.py
The primary entry point for the system. While it runs the full stack by default, it supports several optional arguments for development:

| Command | Logic / Argument | Resulting Action |
|:---|:---|:---|
| `./manage.py` | (No arguments) | Runs `run_runtime()` immediately using live volumes. |
| `./manage.py build` | `build` | Runs `run_build(full=False)`. Re-compiles ROS code. |
| `./manage.py full-build` | `full-build` | Runs `run_build(full=True)`. Cleans system & Re-installs all system dependencies. |


### Interactive Shell
To enter the running container for debugging or manual ROS 2 commands:
```bash
./login.sh
```

### Diagnostics
If hardware is connected but topics are not flowing, run the diagnostic tool from inside the container:

#### After running ./login.sh
```bash
python3 agbot-diagnostic.py
```

![TUI Status.](https://raw.githubusercontent.com/Agroecology-Lab/Open_agbot_devkit_ros/refs/heads/main/assets/Screenshot%20From%202026-01-21%2018-07-45.png)

You can also make it verbose with:
```bash
python3 agbot-diagnostic.py full
```

### Proposed GNSS Topic Mapping: Septentrio vs. Dual u-blox F9P (Moving Base)

| Septentrio Topic | `ublox_dgnss` Topic (Rover-Rover) | Source UBX Message | Functional Description |
| :--- | :--- | :--- | :--- |
| `/pvtgeodetic` | `/ubx_nav_pvt` | **NAV-PVT** | Geodetic position, velocity, and time. |
| `/gpsfix` | `/ublox_nav_sat_fix_hp` | **NAV-PVT + NAV-HPPOSLLH** | High-precision global position fix. |
| `/poscovgeodetic` | `/ubx_nav_cov` | **NAV-COV** | Position and velocity covariance matrix. |
| `/atteuler` | `/ubx_nav_rel_pos_ned` | **NAV-RELPOSNED** | **Heading/Yaw** derived from the relative vector. |
| `/attcoveuler` | `/ubx_nav_rel_pos_ned` | **NAV-RELPOSNED** | Accuracy/Variance of the heading calculation. |
| `/aimplusstatus` | `/ubx_mon_rf` | **MON-RF** | Jamming/Interference monitoring indicators. |


## Sketch of MVP 2026 architecture

### 1. The Lizard Brain (RT Microcontroller)
* **Hardware:** ESP32 MCU.
* **Software:** Lizard DSL.
* **Role:** Hard Real-Time Execution.
* **Function:** Motor PID control and physical safety (bumpers/cliffs).
* **I/O:** 3.3V UART receiving $v, \omega$ via the `teleop_lizard` ROS 2 bridge.

### 2. The Limbic System (Executive)
* **Hardware:** Avaota A1 #1 (Allwinner T527).
* **Software:** ROS 2 Jazzy + `topological_navigation` (AOC branch).
* **Role:** Navigation Executive.
* **Function:** Runs the Topological Navigation stack. UBLOX sensors Manages the move_base sequence and Action on Condition (AOC) logic.
* **I/O:** Connects to u-blox via USB/UART using `ublox_dgnss` node. Translates graph goals into velocity commands for the Lizard Brain.

**<1GbE interconnect between 2&3>**

### 3. The Neo (Perception)
* **Hardware:** Avaota A1 #2 (Allwinner T527 + NPU).
* **Software:** Dockerised ROS 2 Jazzy.
* **Role:** Asynchronous Perception.
* **Function:** NPU-accelerated inference (YOLO/Object tracking) and sensor fusion.
* **Connectivity:** Native Zenoh integration via `rmw_zenoh_cpp`. Publishes environment states and "Conditions" to the Zenoh network.

# Navigation detail for 2026

| Layer          | Component                                      | Role                                                        |
|----------------|------------------------------------------------|-------------------------------------------------------------|
| **Task**       | LCAS topological_navigation                    | "Visit crop rows A, B, C" — graph of named farm locations   |
| **Navigation** | Nav2 (fazenda_completa)                        | Moves between poses                                         |
| **Localisation**| GPS + EKF (navsat_transform + ekf_node)       | Outdoor positioning                                         |
| **Simulation** | Gazebo (sim.launch.py + minha_fazenda.world)   | Simulated environment                                       |

## Sketch of possible eventual ~2027 architecture

### 1. The Lizard Brain (Hardware Abstraction)
* **Hardware:** [STM32 H7 MCU](https://oshwhub.com/6676a/stm32h745zit6_core).
* **Software:** copper-rs.
* **Role:** Hard Real-Time Execution.
* **Function:** Manages motor PID loops and hardware-level safety interlocks.
* **I/O:** Canbus

### 2. The Limbic System (Executive)
* **Hardware:** Avaota A1 #1 (Allwinner T527).
* **Software:** RT kernel, Buildroot `copper-rs`
* **Role:** Deterministic Executive.
* **Function:** UBLOX sensors, Executes Action on Condition (AOC) logic for topological navigation.
* **Data Entry:** Directly consumes Zenoh keys from the Neo board to trigger mission state transitions and motion planning.

| Core(s)   | Role                | Allocation Strategy                                                                 |
| :-------- | :------------------ | :---------------------------------------------------------------------------------- |
| Core 0    | OS / I/O            | Handles kernel house-keeping, SSH, and the 1GbE driver interrupts.                  |
| Core 1    | Zenoh / Neo-link    | Dedicated to the Zenoh router and serializing incoming "Nice-to-Have" data.         |
| Cores 2-6 | The Pilot (Nav)     | This is where the RTK EKF, Path Planner, and Task Graph live.                       |
| Core 7    | The Bridge (Lizard) | Dedicated to SocketCAN and the high-frequency heartbeat to the STM32 (Lizard).      |


**<1GbE interconnect between 2&3>**

### 3. The Neo (Perception)
* **Hardware:** Avaota A1 #2 (Allwinner T527 + NPU).
* **Software:** Dockerised ROS 2 Jazzy & Dockerised CV packages
* **Role:** Asynchronous Perception.
* **Function:** NPU-accelerated inference (YOLO/Object tracking) and sensor fusion.
* **Connectivity:** Native Zenoh integration via `rmw_zenoh_cpp`. Publishes environment states and "Conditions" to the Zenoh network.



## Feldfreund DevKit ROS 
(Below from original Zauberzeug forked repo)

Feldfreund DevKit ROS is a comprehensive ROS2 package that handles the communication and configuration of various Feldfreund components:

- Communication with Lizard (ESP32) to control the Feldfreund
- GNSS positioning system
- Camera systems (USB and AXIS cameras)
- Example UI to control the robot

All launch files and configuration files (except for the UI) are stored in the `devkit_launch` package.

## Components

### DevKit driver

The DevKit driver (based on [ATB Potsdam's field_friend_driver](https://github.com/ATB-potsdam-automation/field_friend_driver)) manages the communication with the ESP32 microcontroller running [Lizard](https://lizard.dev/) firmware - a domain-specific language for defining hardware behavior on embedded systems.

The package provides:

- `config/devkit.liz`: Basic Lizard configuration for DevKit robot
- `config/devkit.yaml`: Corresponding ROS2 driver configuration

Available ROS2 topics:

- `/cmd_vel` (geometry_msgs/Twist): Control robot movement
- `/odom` (nav_msgs/Odometry): Robot odometry data
- `/battery_state` (sensor_msgs/BatteryState): Battery status information
- `/bumper/front_top` (std_msgs/Bool): Front top bumper state
- `/bumper/front_bottom` (std_msgs/Bool): Front bottom bumper state
- `/bumper/back` (std_msgs/Bool): Back bumper state
- `/estop/soft` (std_msgs/Bool): Software emergency stop control
- `/estop/front` (std_msgs/Bool): Hardware front emergency stop state
- `/estop/back` (std_msgs/Bool): Hardware back emergency stop state
- `/configure` (std_msgs/Empty): Trigger loading of the Lizard configuration file

### Camera System

The camera system supports both USB cameras and AXIS cameras, managed through a unified launch system in `camera_system.launch.py` that handles USB cameras, AXIS cameras, and the Foxglove Bridge for remote viewing.

The USB camera system provides video streaming through ROS2 topics using the `usb_cam` ROS2 package. Camera parameters can be configured through `config/camera.yaml`.

The AXIS camera system integrates with the [ROS2 AXIS camera driver](https://github.com/ros-drivers/axis_camera/tree/humble-devel) to support multiple IP cameras with individual streams. Each camera can be configured through `config/axis_camera.yaml`, with credentials managed through `config/secrets.yaml` (template provided in `config/secrets.yaml.template`). The cameras' authentication mode (basic or digest) might need to be configured - see [AXIS Camera Authentication](#axis-camera-authentication) section for details.

The visualization system integrates with [Foxglove Studio](https://foxglove.dev/) for remote camera viewing, supporting compressed image transport. The Foxglove Bridge is accessible via WebSocket connection on port 8765.

### GNSS System

The GNSS system uses the [Septentrio GNSS driver](https://github.com/septentrio-gnss/septentrio_gnss_driver) with the default `config/gnss.yaml` configuration. Available topics:

- `/pvtgeodetic`: Position, velocity, and time in geodetic coordinates
- `/poscovgeodetic`: Position covariance in geodetic coordinates
- `/velcovgeodetic`: Velocity covariance in geodetic coordinates
- `/atteuler`: Attitude in Euler angles
- `/attcoveuler`: Attitude covariance
- `/gpsfix`: Detailed GPS fix information including satellites and quality
- `/aimplusstatus`: AIM+ status information

### DevKit UI

The example UI provides a robot control interface built with NiceGUI, featuring a joystick control similar to turtlesim. It gives you access to and visualization of all topics made available by the DevKit driver, including:

- Robot movement control through a joystick interface
- Real-time visualization of GNSS data
- Monitoring of safety systems (bumpers, emergency stops)
- Software emergency stop control

The interface is accessible through a web browser at `http://<ROBOT-IP>:80` when the robot is running.

<div align="center">
  <img src="assets/DevKitUI.png" alt="Example UI Screenshot" width="500"/>
  <div style="font-size: 0.95em; color: #555; margin-top: 0.5em;">
    Example UI: Control, data, safety, and GPS map in one interface.
  </div>
</div>

## Docker Setup

### Using Docker Compose

1. Build and run the container:

```bash
cd docker
docker-compose up --build
```

2. Run in detached mode:

```bash
docker-compose up -d
```

3. Attach to running container:

```bash
docker-compose exec devkit bash
```

4. Stop containers:

```bash
docker-compose down
```

The Docker setup includes:

- All necessary ROS2 packages
- Lizard communication tools
- Camera drivers
- GNSS drivers

## Connect to UI

To access the user interface (UI), follow these steps:

1. **Connect to the Robot's Wi-Fi:**
   Join the robot's WLAN network.

2. **Open the UI in your browser:**
   Navigate to:

   ```
   http://<ROBOT-IP>:80
   ```

   (Replace `<ROBOT-IP>` with the actual IP address once you have it.)

## Launch Files

The system can be started using different launch files:

- `devkit.launch.py`: Launches all components
- `devkit_nocams.launch.py`: Launches all components without the cameras
- `devkit_driver.launch.py`: Launches only Feldfreund DevKit driver
- `camera_system.launch.py`: Launches complete camera system (USB + AXIS) and Foxglove Bridge
- `usb_camera.launch.py`: Launches USB camera only
- `axis_cameras.launch.py`: Launches AXIS cameras only
- `gnss.launch.py`: Launches GNSS system
- `ui.launch.py`: Launches the example UI node

To launch the complete system:

```bash
ros2 launch devkit_launch devkit.launch.py
```

## AXIS Camera Authentication

The AXIS cameras can be configured to use either digest or basic authentication. To check and configure the authentication mode:

1. Check current authentication settings:

```bash
curl --digest -u root:pw "http://192.168.42.3/axis-cgi/admin/param.cgi?action=list&group=Network.HTTP" | cat
```

2. Switch authentication mode (e.g., from digest to basic):

```bash
curl --digest -u root:pw "http://192.168.42.3/axis-cgi/admin/param.cgi?action=update&Network.HTTP.AuthenticationPolicy=basic" | cat
```

Replace `root:pw` with your camera's credentials and `192.168.42.3` with your camera's IP address. The authentication mode can be set to either `basic` or `digest`. Note that you should always use the `--digest` flag in these commands even when switching to basic auth, as the camera's current setting might be using digest authentication.

## Quickstart guide

### 1. Clone the Repository

```bash
git clone https://github.com/zauberzeug/devkit_ros.git
cd devkit_ros
```

### 2. Validate Configuration

Before building, check and adjust if needed:

1. **ROS2 Configuration** (`devkit_launch/config/devkit.yaml`):
   - Verify `serial_port` matches your setup (default: "/dev/ttyTHS0")
   - Check `flash_parameters` for your hardware (default: "-j orin --nand")

2. **Lizard Configuration** (`devkit_launch/config/devkit.liz`):
   - Verify motor configuration matches your hardware
   - Check pin assignments for bumpers and emergency stops
   - Adjust any other hardware-specific settings

### 3. Build with Docker

```bash
./docker.sh u
```

### 4. Send Lizard Configuration

Once the system is running:

- Use the "Send Lizard Config" button in the UI
- Or use the `/configure` topic in ROS2

### 5. Ready to Go

Check the UI at `http://<ROBOT-IP>:80` to control and monitor your robot.

## Future features

This repository is still work in progress. Please feel free to contribute or reach out to us, if you need any unimplemented feature.

- Complete tf2 frames
- Handle camera calibrations
- Robot visualization
