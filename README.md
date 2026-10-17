# Sowbot (ROS 2 stack)
<!-- CI & Test Status -->
[![Industrial CI](https://github.com/Agroecology-Lab/Sowbot/actions/workflows/industrial_ci.yml/badge.svg?branch=caatinga-dev)](https://github.com/Agroecology-Lab/Sowbot/actions/workflows/industrial_ci.yml?query=branch%3Acaatinga-dev)
[![Run Tests](https://github.com/Agroecology-Lab/Sowbot/actions/workflows/test.yml/badge.svg)](https://github.com/Agroecology-Lab/Sowbot/actions/workflows/test.yml)
[![CodeQL](https://github.com/Agroecology-Lab/Sowbot/actions/workflows/github-code-scanning/codeql/badge.svg)](https://github.com/Agroecology-Lab/Sowbot/actions/workflows/github-code-scanning/codeql)
[![CodeQL Code Quality](https://github.com/Agroecology-Lab/Sowbot/actions/workflows/github-code-quality/codeql/badge.svg)](https://github.com/Agroecology-Lab/Sowbot/actions/workflows/github-code-quality/codeql)
[![Docs](https://github.com/Agroecology-Lab/Sowbot/actions/workflows/documentation.yml/badge.svg)](https://github.com/Agroecology-Lab/Sowbot/actions/workflows/documentation.yml)

<!-- Code Coverage -->
[![Codecov](https://codecov.io/gh/Agroecology-Lab/Sowbot/branch/caatinga-dev/graph/badge.svg)](https://codecov.io/gh/Agroecology-Lab/Sowbot/branch/caatinga-dev)

An open-source, containerised ROS2 Jazzy stack for autonomous agricultural robotics.
This repository provides the drivers and orchestration for the Sowbot platform, featuring RTK-GNSS localisation and STM32-based hardware control.

[Quick start](#quick-start)

[Docs](https://agroecology-lab.github.io/Sowbot_Data/python_api.html)
```
+---------------------------------------------------------+
|               ROS 2 DUAL SBC Open Core                  |
| +----------------------+     +----------------------+   |
| |       NEO SBC        | Gig |      LIMBIC SBC      |   |
| | - Camera Streams     | Eth | - NiceGUI Dashboard  |   |
| |   (YOLO / ExG)       |<===>| - Nav2/LCAS TopoNav  |   |
| | - Visual Servoing    |     | - FusionCore UKF     |   |
| | - Crop Processing    |     | - devkit_mavlink     |   |
| | - High-Throughput    |     |   (cmd_vel to MAVLink|   |
| |   Perception         |     |   zero if stale)     |   |
| +----------------------+     | - Dual F9P RTK (USB) |   |
|                              +----------+-----------+   |
+---------------------------v-----------------------------+
                            |
                    Serial MAVLink (UART)
        baud: MAVLINK_BAUD here, SERIALn_BAUD on the FC
                            |
+---------------------------v-----------------------------+
|              ARDUPILOT FLIGHT CONTROLLER                |
|    LEVIA-H7 running ArduRover, GUIDED mode (bring-up)   |
|    SUPERVISORY: not the rated stop function             |
|---------------------------------------------------------|
| - Motion execution       - EKF3; planned GPS_INPUT     |
| - Geofence (GUIDED)         from FusionCore            |
| - Failsafes: GCS/GPS     - Arming state machine         |
|   loss, EKF variance     - Pre-arm checks               |
+---------------------------+-----------------------------+
                            |
                    DroneCAN / CAN bus
              (motor driver choice open, O31)
                            |
+---------------------------v-----------------------------+
|            MOTOR CONTROLLERS (48V traction)             |
|---------------------------------------------------------|
| Differential-drive tracked platform                     |
| Traction power passes through the SW180 contactors      |
| of the hardwired safety loop below                      |
+---------------------------------------------------------+

+---------------------------------------------------------+
|      HARDWIRED 24V SAFETY LOOP (rated stop function)    |
| E-stops, bumpers, Inxpect radar + C203A -> 2x SW180     |
| in series on the 48V bus. Independent of ROS,           |
| ArduPilot and the SBCs. Not yet complete: see           |
| safety_roadmap.md (O7, O8, O14)                         |
+---------------------------------------------------------+
```

Controller status: ArduPilot on LEVIA-H7 is the current controller path and is in bring-up (the board is unvalidated).
ESP32/Lizard is legacy.
The 24V loop, not the controller, is the rated stop function.

Reference open hardware stack(s) under development at [Sowbot.co.uk](https://sowbot.co.uk/hardware) Contact: [sowbot.co.uk/contact](https://sowbot.co.uk/contact/)

Development is led by the <a href="https://agroecologylab.org.uk" target="_blank">Agroecology Lab</a> building on the core developed by <a href="https://zauberzeug.com/" target="_blank">Zauberzeug</a>.

We have a [Online version of swath planner](https://agroecology-lab.github.io/feldfreund_devkit_ros/)

**Collaborators welcome.**
See [CONTRIBUTING.md](CONTRIBUTING.md).

**Milestone — multi-row mission following validated in Gazebo:**

[![Multi-row mission following in Gazebo](https://img.youtube.com/vi/A0CVNcp19vU/0.jpg)](https://www.youtube.com/watch?v=A0CVNcp19vU)

(click above image to play video)


[![Architecture diagram of agroecology-lab/Sowbot](https://gitdiagram.com/agroecology-lab/Sowbot/diagram.png)](https://gitdiagram.com/agroecology-lab/Sowbot?utm_source=readme&utm_medium=picture)

## Sowbot Roadmap

Related [Safety Roadmap](https://github.com/Agroecology-Lab/Sowbot/blob/caatinga-dev/safety_roadmap.md)

| # | Feature | Description | Status | TRL | Phase |
|---|---------|-------------|--------|-----|-------|
| **FOUNDATION** | | | | | **2025** |
| F1 | Containerised deployment | Full ROS 2 Jazzy stack managed via Docker and `manage.py`. Live volume mapping to `/workspace`. Build, full-build, and `+sim` build modes. | Done | 6 | 2025 |
| F2 | Stable device addressing | `fixusb.py` with Jetson/generic architecture detection, kernel `low_latency` mode, and udev symlink generation. Writes `.env` consumed by all launch files. Note: `run_neo()` never calls `fixusb.py` so `.env` is not written for neo-only runs. | Done | 6 | 2025 |
| F3 | Teleop dashboard | NiceGUI web cockpit on `:80`. Three-tab interface: **Nav** (joystick, e-stop, topo map, node-drop, track mode), **Mission** (fields2cover corner entry, F2C row plan generator, reorderable mission queue), **System** (telemetry, safety indicators, GPS leaflet map). | Done | 6 | 2025 |
| F4 | ublox DGNSS driver | Dual F9P moving-base configuration with dynamic port assignment via `fixusb.py`. | Done | 6 | 2026 |
| F5 | Diagnostics TUI | `agbot-diagnostic.py` terminal status view of all hardware topics. Run inside container via `login.sh`. | Done | 6 | 2025 |
| **MVP FIELD** | | | | | **2026** |
| M1 | Agri Open Core platform integration | Demonstrates AOC platform abstraction on affordable ARM hardware accessible to smallholders. | Done | 3 | 2026 |
| M2 | Topological navigation + Nav2 | LCAS `topological_navigation` (`aoc_refactor` branch) building in Dockerfile. RViz visualisation confirmed working. Self-contained `navigation2.py` with A\* route planning, explicit state machine, and `row_traversal` / `NavigateToPose` / `goal_align` edge actions. `fake_nav2_server` with `/limbic_row_follow` stub enables full pipeline testing in sim. `sim_nav.launch.py` now routes through real Nav2. **Multi-row mission following (entry/exit node pairs across boustrophedon rows) validated end-to-end in Gazebo.** Pending: Jazzy field validation on hardware. | Sim only. Blocked: hardware field validation | 4 | 2026 |
| M2a | fusioncore Nav2 bridge | `fusioncore_node` launched via `devkit.launch.py`. Nav2 topic remapping shim wired. Pending: end-to-end test on live hardware. | Blocked: live hardware test | 4 | 2026 |
| M3 | Open-field row-crop scenario | Live node-drop in UI. F2C row plan generator implemented in `PlanApplicationService` (corners → swaths → topo rows via `run_f2c()`). YAML written to `/workspace/maps/`, `switch_topological_map` with fallback. Topo map auto-generated at container start. Multi-row traversal across generated swaths confirmed in sim. [Web version of planner available](https://agroecology-lab.github.io/feldfreund_devkit_ros/) Pending: tmap2 authoring from real field survey; F2C obstacle costmap integration. | Sim only. Blocked: real-field tmap2 survey, F2C obstacle costmap | 4 | 2026 |
| M4 | RTK-GNSS localisation | Full pipeline implemented: dual F9P, shims, UKF fusion, NTRIP. Lever-arm offsets in `fusioncore.yaml` are zeroed placeholders (commented `# measured` TODOs). Pending: antenna lever-arm measurement, live hardware test. Do not quote accuracy before the lever arms are measured. | Blocked: lever-arm measurement, live hardware test | 3 | 2026 |
| M5 | Dual-SBC ROS 2 stack | `manage.py` detects crossover interface, builds `CYCLONEDDS_URI` peer config and injects into Docker. `neo.launch.py` and `devkit.launch.py` finalised for Limbic+Neo split. DDS peer path is the active one. | Implemented. No hardware test recorded | 3 | 2026 |
| M6 | Gazebo simulation | `sowbot_sim.launch.py` + `sim_nav.launch.py` fully restructured. `kill_fake_nav2_on_clock` implemented. `use_sim_time=True` now threaded through topo stack. Gazebo does not model track slip, so these results do not carry over to the tracked hardware. **Multi-row mission following demonstrated end-to-end (see video above).** | Done | 4 | 2026 |
| M7 | Sentor safety monitoring | `sowbot_monitor.yaml` fully authored (e-stop, bumpers, battery, camera, odom, neo_vision heartbeat, node monitors). `sentor_node.py` wired into `devkit.launch.py`. Pending: smoke-test on live hardware; battery voltage cutoff needs field confirmation (`# TODO: CONFIRM` in YAML). GNSS fix and heading monitors report only; no response is defined yet. | Blocked: hardware smoke test, battery cutoff | 3 | 2026 |
| M8 | Visual crop-row navigation | `sowbot_row_follow` package implemented. ExG+Otsu, visual servo, `limbic_row_follow_node.py` as Nav2 action server. Cancel and heartbeat-loss safety. TSM row-swap hold with 6-second debounce implemented. Tested and iterated against a lettuce crop in the UK. | Tested on lettuce. No metrics published | 4 | 2026 |
| M9 | ArduPilot on LEVIA-H7 controller path | Replaces ESP32/Lizard. `devkit_mavlink_bridge` outbound half works and commands zero when `cmd_vel` is stale; inbound FusionCore to `GPS_INPUT` is not built. LEVIA-H7 is unvalidated and has no confirmed ArduPilot hwdef (safety_roadmap.md O31). | WIP, bring-up | 2 | 2026 |
| **PRODUCTION** | | | | | **~2027** |
| P1 | Phase 1: dev platform (current) | Controller: ArduPilot on LEVIA-H7 (M9); ESP32/Lizard is legacy. Supervised-only until the 24V stop loop, radar validation and beacon are done (safety_roadmap.md O7, O8, O14). Audience: university labs, ag-tech researchers, software startups. No compliance claimed, standards used as informal design reference only, user's own risk per README. Product status [Partly completed machinery](https://github.com/Agroecology-Lab/Sowbot/blob/caatinga-dev/safety_roadmap.md#product-classification) | In progress. Blocked: O7, O8, O14 | 4 | 2026 |
| P2 | Phase 2: OEM modular subsystems | Packages the Phase 1 stack ([Ardupilot on Levia STM32 H7](https://github.com/piecol/LEVIA-H7), M9) for integrators. Audience: startups integrating Sowbot's core. No third-party certification needed yet, formal internal assessment only. Can offer [PLd hardware subsystem](https://github.com/Agroecology-Lab/Sowbot/blob/caatinga-dev/safety_roadmap.md) | Research | 2 | 2027 |
| P3 | Phase 3: commercial sale to farmers [(Safety sub-systems hardened)](https://github.com/Agroecology-Lab/Sowbot/blob/caatinga-dev/safety_roadmap.md) | Full ISO 18497 compliance, ISO 13849 PLc certification (or equivalent), third-party safety audit, field trial history, and an insurance/liability structure. All required before sale, none yet scoped. | Planned | 1 | 2027+ |
| P4 | CANopen bus | ISO 11898 FDCAN at 500 kbit/s / 2 Mbit/s. lely-core CANopen master on T527 native M_CAN. DSP402 drive profile. | Research | 2 | 2027 |
| P5 | RT kernel + core isolation | PREEMPT_RT on Limbic T527. `isolcpus=4-7`, RTK EKF on core 2 (SCHED_FIFO 60), AOC nav on cores 4-6, watchdog on core 5. GbE/CAN IRQ pinned to core 0. | Planned | 2 | 2027 |
| P6 | ROFS image | Read-only rootfs, Ubuntu Noble minimal or Yocto with RT kernel, pre-built LCAS topo nav, Nav2, `rmw_zenoh_cpp`. Immutable field deployment. | Research | 1 | 2027 |
| **END-EFFECTORS** | | | | | **TBD** |
| E1 | Delta weeding module | Open-Weeding-Delta precision mechanical weeding. CANopen actuator node on delta controller. | Research | 1 | TBD |
| E2 | LASER weeding module | Laudando LASER integration. Requires E-Stop interlocking with CANopen safety chain. | Research | 1 | TBD |
| **DATASETS & COLLABORATION** | | | | | **Ongoing** |
| D1 | UK open-field dataset | Field imagery and GNSS logs from UK agroecological farm conditions. CC licence. | Planned | 2 | 2026 |
| D2 | Caatinga biome dataset | Semi-arid row-crop imagery from caatingarobotics. Validated on T527 AIPU. | Active | 5 | 2026 |

<sub>TRL = Technology Readiness Level (1–9, ESA/NASA scale): 1–2 concept/formulation, 3 proof of concept, 4 validated in lab/simulation, 5 validated in relevant (non-lab) environment, 6 demonstrated in relevant environment, 7 operational prototype, 8 qualified system, 9 field-proven.
Self-assessed per feature, not a formal review — adjust as needed.</sub>

### Field results

None published yet.
Simulation results are not evidence for the tracked hardware (Gazebo does not model slip).
Once hardware runs start, each will report: cross-track error (RMS and maximum), share of time in RTK fixed, interventions per hour, and measured stopping distance.
The progression from bench to supervised rows, with exit criteria for each step, is in [docs/field-validation.md](docs/field-validation.md).

### Collaboration

This project is built on and aims to maintain upstream compatibility with [zauberzeug/feldfreund\_devkit\_ros](https://github.com/zauberzeug/feldfreund_devkit_ros).

High Level navigation is developed from the work of [Lincoln Centre for Autonomous Systems (LCAS)](https://lcas.lincoln.ac.uk) as part of the [Agri-OpenCore](https://agri-opencore.org) open ROS 2 ecosystem for agricultural robotic

Simulation configuration is partially derived from work by [caatingarobotics](https://github.com/joaodemouragy-hash/caatingarobotics),


# ⚠️ CRITICAL SAFETY WARNING:

 This software is under active development and may be broken at any given moment.
For a stable reference implementation see the upstream Zauberzeug project.

**THIS SOFTWARE COULD CONTROL PHYSICAL HARDWARE CAPABLE OF PRODUCING SIGNIFICANT KINETIC FORCE.**

1. **EXPERIMENTAL STATUS**: This branch ('caatinga-dev') contains experimental code generated and
   refined with AI assistance.
   It has NOT undergone full-scale field validation.
2. **STATUTORY NOTICE (UK)**: Usage of this software is at the user's sole risk.
   While
   standard open-source licenses apply, users are reminded that operating agricultural
   robotics requires a professional duty of care.
3. **MANDATORY HARDWARE SAFETY**: Under no circumstances should this software be used
   to control a robot of any size without an independent safety subsystem.
   Software-based stops (such as /estop/soft) are NOT a substitute for
   Category 0 or 1 hardware safety stops.
4. **CURRENT STATUS**: the hardwired 24V stop loop is still being built and the radar is not yet validated
   on the vehicle (see [safety_roadmap.md](safety_roadmap.md), O7, O8, O14).
   Operation requires the hard-wired 24V loop, including the physical E-stop path, to be built and tested, or a separately validated independent stop method; otherwise operation is prohibited.
   Until O7, O8 and O14 close, any permitted operation is supervised-only, with a competent person within reach of a physical E-stop (or the separately validated stop control), nobody working near it, and no safety claim made.
5. **NO LIABILITY**: To the extent permitted by the laws of England and Wales, the
   contributors exclude all liability for property damage, crop loss, or indirect
   consequential damages.

### Health Warning

This repo may contain traces of LLM slop, We've done our best to mitigate this.
If you are allergic to slop, please help us refactor.

## Quick Start

### Supported SBC configuration

- [Avaota A1](https://pine64.com/product/yuzuki-avaota-a1-single-board-computer-4gb-32gb/)
- [Armbian](https://armbian.com/boards/avaota-a1) (note flash with official Armbian flasher)

### 0. Install dependencies

#### Linux
- [Git](https://github.com/git-guides/install-git)
- [Docker](https://docs.docker.com/engine/install/debian/#install-using-the-repository)
- ```sudo apt install python3-serial setserial v4l-utils```
- ```sudo apt install -y pipx && pipx install 'glances[web]' && pipx ensurepath && (nohup ~/.local/bin/glances -w --bind 127.0.0.1 > /tmp/glances.log 2>&1 < /dev/null &)```

#### Mac
- xcode-select --install
- [Git](https://github.com/git-guides/install-git)
- [Docker](https://docs.docker.com/engine/install/)
- & launch Gazebo in a browser from the WebUI.

#### Windows?
- [Git](https://github.com/git-guides/install-git)
- [Docker](https://docs.docker.com/engine/install/)
- Untested!

### 1. Clone the Repository
Open a terminal on your host machine and download the workspace:
```bash
git clone -b caatinga-dev https://github.com/Agroecology-Lab/Sowbot.git
cd Sowbot
```


### 2. Build & Launch
Use the management script to build the ROS 2 workspace and launch the robot stack.
This script automatically handles hardware discovery and port permissions:
```bash
./manage.py full-build
xhost +local:docker
./manage.py
```
Access http://localhost to access the WebUI

**The dashboard has a joystick and a soft e-stop and listens on port 80 of the robot.**
Set `SOWBOT_UI_PASSWORD` in `.env` to require a password (HTTP Basic, user name ignored).
Without it, anyone on the robot's network can drive it.
Basic auth is not encrypted, so also keep the robot's Wi-Fi private.
The container runs privileged on the host network.



#### Linux

If you're getting this error:
```
docker: Error response from daemon: could not select device driver "" with capabilities: [[gpu]]
```
You'll need to install the [Nvidia Container runtime](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).

#### Gazebo

If you'd like to use Gazebo then add a +sim argument to your build instruction

```bash
./manage.py full-build +sim
xhost +local:docker
./manage.py
```

#### Optical flow sensor (PMW3901 / PAA5100)

[`optical_flow_ros`](https://github.com/adityakamath/optical_flow_ros) is built into the image (Dockerfile Stage 8, pinned by `OPTICAL_FLOW_ROS_COMMIT`).
Its pip dependencies (`pmw3901`, `gpiod`, `gpiodevice`) are installed at pinned versions in Stage 5.2b.

Run inside the container:
```bash
ros2 launch devkit_bringup optical_flow.launch.py
```

Do not use upstream's `optical_flow_launch.py`: it broadcasts `odom` -> `base_link`, which FusionCore owns.

- Sensor parameters: `config/sensor_params.yaml` in the upstream package (`board`, `spi_nr`, `spi_slot`, `rotation`, `z_height`, `sensor_timeout`).
  `src/devkit_bringup/config/optical_flow.yaml` is loaded after it and sets `publish_tf: false`.
  Put local driver settings there, not in the cloned source.
- The launch remaps `odom` to `flow_odom` and publishes `/diagnostics`.
- `flow_odom` is not consumed by FusionCore (its inputs are hardcoded to `/gnss/fix` and `/odom/wheels`).
- Upstream tested on Raspberry Pi (SPI).
  Not tested here on the supported SBCs.
- Requires `/dev/spidev*` on the host; the container runs privileged with `/dev` mounted.

Known limitations of the pinned driver, not fixed here:
- `flow_odom` is not fit for fusion.
  The orientation is left unset (0, 0, 0, 0), every covariance is zero and angular velocity is zero.
  Do not feed it to an estimator without fixing the message.
- `/diagnostics` reports OK whenever the node runs.
  A sensor timeout is published as zero motion, so it is not a health signal.
- Each read can block for up to `sensor_timeout` (0.5 s) inside a 100 Hz timer.
  A stationary sensor therefore publishes slower than 100 Hz, and the linear velocity (distance divided by the nominal 0.01 s) is wrong whenever a callback runs late.

## Management & Tools

### manage.py
The primary entry point for the system.
While it runs the full stack by default, it supports several optional arguments for development:

| Command | Logic / Argument | Resulting Action |
|:---|:---|:---|
| `./manage.py` | (No arguments) | Runs `run_runtime()` immediately using live volumes.|
| `./manage.py build` | `build` | local-only, fully cached including external clones |
| `./manage.py build +pull` | `build` | Re-clones the 10 external repos, keeps apt/pip/local layers cached |
| `./manage.py build +pull +sim` | `build ` | Same, plus INSTALL_SIM=true |
| `./manage.py full-build` | `full-build` | Runs `run_build(full=True)`. Cleans system & Re-installs all system dependencies. |
| `./manage.py neo` | `neo` | Runs `neo`. Runs only the line following code for the second 'neo'(cortex) perception SBC. Then check `http://localhost:8081` (web_video_server; 8080 is the Medkit gateway) |
| `./manage.py neo-tsm` | `neo-tsm` | Runs `neo-tsm`. Runs experimental line following informed by [Vision based Crop Row Navigation under Varying Field Conditions in Arable Fields - Rajitha de Silva1, Grzegorz Cielniak2 and Junfeng Gao](https://arxiv.org/pdf/2209.14003). Then check `http://localhost:8081` (web_video_server; 8080 is the Medkit gateway) |
| `./manage.py pull-caatinga` | `pulls & builds vision pipeline` | Requires the container to already be running |


### One-command sim launch (TMuLE)

The simulation launch accepts `use_camera:=false` to disable the simulated camera:

```bash
ros2 launch devkit_bringup sowbot_sim.launch.py use_camera:=false
```

Use `camera_width`, `camera_height`, and `camera_rate` to override the default 320×240 image at 10 Hz.
These arguments also work with `ros2 launch devkit_simulation sim.launch.py`.

Whilst you can launch Gazebo and other tools from the webui, some may prefer the terminal, this makes terminal use a bit easier.

[TMuLE](https://github.com/marc-hanheide/TMuLE) brings the whole row-following sim up in a single `tmux` session — one window per process — instead of running the three launch steps by hand in separate terminals:

```bash
tmule -c tmule/row_follow_sim.yaml launch
tmux attach -t row_follow_sim
```

That starts `./manage.py --sim` (nav stack + Nav2 + UI), Gazebo (`sowbot_sim.launch.py`) and the crop-row CV node (`crop_row_nav.launch.py`), each in its own tmux window.
`launch` returns immediately and leaves the session detached, so attach to watch the panes come up (and to type the `nav_stack` sudo password).

Stop the stack with `tmule -c tmule/row_follow_sim.yaml stop`; re-attach later with `tmux attach -t row_follow_sim`.

> If a runtime container is **already running**, the `nav_stack` window will fail on a `docker run` name collision.
> Either `docker stop sowbot_runtime` first, or launch just the windows that reuse the container: `tmule -c tmule/row_follow_sim.yaml launch -w gazebo`.

#### tmux basics

The stack runs in a *detached* tmux session named `row_follow_sim`, so closing your terminal does not kill it — **detaching is not stopping**.
The session holds one window per sub-system (plus a stray `0: bash` that tmux always creates):

```
0: bash   1: nav_stack   2: gazebo   3: crop_row
```

Every shortcut starts with the prefix `Ctrl-b` — press and release it, then the next key:

| Keys | Resulting Action |
|:---|:---|
| `Ctrl-b` `d` | Detach — leaves the whole stack running in the background |
| `Ctrl-b` `w` | Interactive window picker (easiest way to move around) |
| `Ctrl-b` `1` / `2` / `3` | Jump straight to `nav_stack` / `gazebo` / `crop_row` |
| `Ctrl-b` `n` / `p` | Next / previous window |
| `Ctrl-b` `[` | Scroll back through ROS log output — arrows/PgUp, `q` to exit |
| `Ctrl-b` `z` | Zoom the current pane fullscreen (press again to unzoom) |
| `Ctrl-b` `?` | List every binding |

```bash
tmux ls                              # list sessions
tmux attach -t row_follow_sim        # attach
tmux kill-session -t row_follow_sim  # nuke the session
```

Scrolling back (`Ctrl-b` `[`) is the one you'll reach for most — it's how you read ROS output that has already scrolled past.
For mouse-wheel scrolling instead, add `set -g mouse on` to `~/.tmux.conf`.

Scenario configs live in [`tmule/`](tmule/) — see [`tmule/README.md`](tmule/README.md) to tweak launch arguments or add your own scenario.

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

## Sketch of MVP 2026 architecture

### 1. The Flight Controller (RT Microcontroller)
* **Hardware:** [LEVIA-H7](https://github.com/piecol/LEVIA-H7) (STM32H743).
  Unvalidated, in bring-up.
* **Software:** ArduPilot Rover, GUIDED mode.
* **Role:** Motion execution and supervisory failsafes (geofence, GCS/GPS loss, EKF variance).
* **I/O:** MAVLink serial from the Limbic SBC via `devkit_mavlink_bridge`; CAN for motor drivers (driver choice open).
* **Not the rated stop function.**
  Stopping on a bumper, radar or E-stop is done by the hardwired 24V loop, independent of this controller (see [safety_roadmap.md](safety_roadmap.md)).
* **Legacy:** ESP32 + Lizard DSL (3.3V UART via the `teleop_lizard` bridge, see [docs/lizard-firmware-setup.md](docs/lizard-firmware-setup.md)).
  Kept for existing hardware, no new features.

### 2. The Limbic System (Executive)
* **Hardware:** Avaota A1 #1 (Allwinner T527).
* **Software:** ROS 2 Jazzy + `topological_navigation` (AOC branch).
* **Role:** Navigation Executive.
* **Function:** Runs the Topological Navigation stack.
  UBLOX sensors Manages the move_base sequence and Action on Condition (AOC) logic.
* **I/O:** Connects to u-blox via USB/UART using `ublox_dgnss` node.
  Translates graph goals into velocity commands for the flight controller.

**<1GbE interconnect between 2&3>**

### 3. The Neo (Perception)
* **Hardware:** Avaota A1 #2 (Allwinner T527 + NPU).
* **Software:** Dockerised ROS 2 Jazzy.
* **Role:** Asynchronous Perception.
* **Function:** NPU-accelerated inference (YOLO/Object tracking) and sensor fusion.
* **Connectivity:** Native Zenoh integration via `rmw_zenoh_cpp`.
  Publishes environment states and "Conditions" to the Zenoh network.

                          |

## Sketch of possible eventual ~2027 architecture

Research only (TRL 1 to 2 in the roadmap above), not a commitment.
It would replace ArduPilot on the controller, which is the current path.

### 1. The Real-Time Controller (Hardware Abstraction)
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
| Core 7    | The Bridge (controller) | Dedicated to SocketCAN and the high-frequency heartbeat to the STM32.            |


**<1GbE interconnect between 2&3>**

### 3. The Neo (Perception)
* **Hardware:** Avaota A1 #2 (Allwinner T527 + NPU).
* **Software:** Dockerised ROS 2 Jazzy & Dockerised CV packages
* **Role:** Asynchronous Perception.
* **Function:** NPU-accelerated inference (YOLO/Object tracking) and sensor fusion.
* **Connectivity:** Native Zenoh integration via `rmw_zenoh_cpp`.
  Publishes environment states and "Conditions" to the Zenoh network.


## Licenses & papers

# Sowbot — Dependency Licence Audit

| Component | Source | Licence | Commercial use | Academic Paper | Notes |
|---|---|---|---|---|---|
| **Sowbot** (this repo, formerly feldfreund_devkit_ros) | Agroecology-Lab/Sowbot | MIT (©Zauberzeug GmbH & Agroecology Lab) | ✅ | *None (Local Project)* | Derivative of upstream field-friend; retain Zauberzeug notice |
| **devkit_driver** | local | MIT (©ATB) | ✅ | *None (Utility Driver)* | Retain ATB copyright notice |
| **devkit_ui** | local | MIT (©Agroecology Lab) | ✅ | *None (UI Extension)* | |
| **devkit_bringup** | local | MIT | ✅ | *None (Config/Launch)* | |
| **sowbot_row_follow** | caatingarobotics | BSD-2-Clause | ✅ | [de Silva et al., 2024](https://arxiv.org/abs/2209.14003) | Triangle Scan Method (TSM) for visual crop row navigation; ©PRBonn + ©Agroecology Lab |
| **caatingarobotics** *(devkit_simulation, caatinga_nav, caatinga_vision)* | [samuk/caatingarobotics](https://github.com/samuk/caatingarobotics) (cloned in the Dockerfile); upstream work by [joaodemouragy-hash/caatingarobotics](https://github.com/joaodemouragy-hash/caatingarobotics) | Apache-2.0 | ✅ | *None (Fork Infrastructure)* | row_follow is the BSD-2-Clause exception, above |
| **topological_navigation** | LCAS (`aoc_refactor`) | Apache-2.0 (©LCAS) | ✅ | [Fentanes et al., 2015](https://www.researchgate.net/publication/282752920_Now_or_later_Predicting_and_maximising_success_of_navigation_actions_from_long-term_experience) | High-level planner; derived from the EU STRANDS long-term autonomy project framework |
| **fusioncore** | manankharwar | Apache-2.0 | ✅ | [Kharwar, 2026](https://arxiv.org/abs/2605.25239) | GNSS fusion; patched in-build; retain NOTICE if present |
| **Forest3D** | unitsSpaceLab | GPL (©UNITS Space Lab) | Do not ship in final product | [Cottiga,S., Bourr, K., & Seriani, S. (2026)](https://kbourr.com/#publications)| Sim-only — 3D forestry /agriculture simulation environment |
| **ublox_dgnss** | aussierobots | Apache-2.0 | ✅ | *None (Hardware Driver)* | GNSS driver |
| **sentor** | LCAS (fork of francescodelduchetto/sentor) | MIT | ✅ | *None (Monitoring Tool)* | Topic- and node-monitoring health node |
| **ros2graph_explorer** | nilseuropa | BSD-3-Clause | ✅ | *None (Debug Tool)* | Dev/debug graph inspector |
| **optical_flow_ros** | adityakamath | Apache-2.0 | ✅ | *None (Hardware Driver)* | PMW3901 / PAA5100 optical flow sensor driver |
| **Fields2Cover v2.0.0** | Fields2Cover | BSD-3-Clause | ✅ | [Mier et al., 2023](https://doi.org/10.1109/LRA.2023.3248439) | Built from source; pulls OR-tools (Apache-2.0) + GDAL (MIT) |
| **lizard** | Agroecology-Lab | MIT (©Zauberzeug GmbH) | ✅ | *None (Firmware Tool)* | ESP32 tooling (legacy controller path); retain Zauberzeug notice |
| **YOLOX 0.3.0** + yolox_nano weights | Megvii | Apache-2.0 | ✅ | [Ge et al., 2021](https://arxiv.org/abs/2107.08430) | Confirm weights terms for commercial use |
| **PyTorch (CPU)** | Meta | BSD-3-Clause | ✅ | [Paszke et al., 2019](https://papers.nips.cc/paper/9015-pytorch-an-imperative-style-high-performance-deep-learning-library) | Core machine learning runtime engine |

## Upstream documentation

The original Feldfreund DevKit text (components, UI, camera and AXIS setup) is kept in [docs/upstream-feldfreund-readme.md](docs/upstream-feldfreund-readme.md).
Parts of it do not apply to Sowbot, see the note at the top of that file.
