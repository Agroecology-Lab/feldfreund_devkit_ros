# sowbot_row_follow

Monocular RGB crop-row following node for **Sowbot**, derived from
[visual-multi-crop-row-navigation](https://github.com/Agricultural-Robotics-Bonn/visual-multi-crop-row-navigation)
(Agricultural-Robotics-Bonn, BSD-2-Clause).

Strips the original package to a single forward-facing RGB camera and publishes
lateral offset + heading error into the Sowbot AOC navigation layer.

---

## Package layout

```
sowbot_row_follow/
├── sowbot_row_follow/
│   └── crop_row_node.py      # Main ROS 2 node
├── launch/
│   └── crop_row_nav.launch.py
├── config/
│   └── crop_row_params.yaml  # All tunable parameters (measure camera on your robot first)
├── package.xml
├── setup.py
└── setup.cfg
```

---

## Topics

| Direction | Topic | Type | Notes |
|-----------|-------|------|-------|
| Subscribes | `/caatinga_vision/row_nav/image_raw` | `sensor_msgs/Image` | From `camera_source_node` |
| Publishes | `/aoc/conditions/row_offset` | `std_msgs/Float32` | Lateral offset \[-1, 1\] |
| Publishes | `/aoc/conditions/row_heading_error` | `std_msgs/Float32` | Heading error (rad) |
| Publishes | `/aoc/heartbeat/neo_vision` | `std_msgs/Bool` | Perception heartbeat (M10) |
| Publishes | `/caatinga_vision/row_nav/debug_image` | `sensor_msgs/Image` | Annotated frame (Foxglove) |
| Publishes (optional) | `/cmd_vel` | `geometry_msgs/Twist` | Enable with `use_direct_cmd_vel: true` |

---

## Quick start

### Build

```bash
# From workspace root
colcon build --packages-select sowbot_row_follow
source install/setup.bash
```

### Launch

```bash
# Default (direct cmd_vel for field testing)
ros2 launch sowbot_row_follow crop_row_nav.launch.py

# AOC production mode
ros2 launch sowbot_row_follow crop_row_nav.launch.py use_direct_cmd_vel:=false

# Override camera index
ros2 launch sowbot_row_follow crop_row_nav.launch.py camera_index:=2
```

---

## Camera calibration (mandatory before field use)

Edit `config/crop_row_params.yaml` and set:

| Parameter | What to measure |
|-----------|----------------|
| `camera_height_m` | Tape measure from camera lens to soil surface (metres) |
| `camera_tilt_deg` | Angle below horizontal (downward = negative, e.g. `-80.0`) |

These feed directly into the Cherubini & Chaumette interaction matrix.
Wrong values produce a systematic angular bias.

---

## Detection tuning

Watch `/caatinga_vision/row_nav/debug_image` in Foxglove while adjusting:

| Parameter | Effect |
|-----------|--------|
| `n_scan_windows` | More columns → finer detection, more CPU |
| `window_width` | Should span roughly one crop row |
| `min_contour_area` | Increase to filter soil/stone noise |

---

## Attribution

Derived from:
- Ahmadi *et al.*, "Towards Autonomous Crop-Agnostic Visual Navigation in Arable Fields," arXiv:2109.11936, 2021.
- Ahmadi *et al.*, ICRA 2020.
- Original code: https://github.com/Agricultural-Robotics-Bonn/visual-multi-crop-row-navigation (BSD-2-Clause)
- ROS 2 port & Sowbot adaptation: Agroecology Lab Ltd, 2026.
