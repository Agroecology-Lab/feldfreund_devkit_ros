# Issue #8 Runbook and Evidence

## Purpose
This runbook captures the exact commands and key logs used for Issue #8.
The goal is reproducibility for the stack comparison and recommendation.
This document is intentionally command-first and log-backed.

## Environment snapshot
Timestamp (UTC): `2026-02-20T22:31:50Z`.
Host OS: `Ubuntu 22.04.5 LTS`.
Python: `3.10.12`.
ROS in host path: `ROS 2 Humble`.
Important note: issue target is Ubuntu 24.04 and ROS 2 Jazzy.
This evidence is a proxy pass and must be rerun on the target environment.

## Repository setup commands
```bash
mkdir -p /tmp/issue8_eval
cd /tmp/issue8_eval

git clone --depth 1 https://github.com/EasyNavigation/EasyNavigation.git
git clone --depth 1 --branch aoc_refactor https://github.com/LCAS/topological_navigation.git topological_navigation_aoc_refactor
git clone --depth 1 https://github.com/samuk/caatingarobotics.git
```

Commits used:
```bash
cd /tmp/issue8_eval/EasyNavigation && git rev-parse HEAD
# ce3fedba579f9bb24bf3fb59feb51e92e629af4d

cd /tmp/issue8_eval/topological_navigation_aoc_refactor && git rev-parse HEAD
# cc9869223fa4bf568881e2dfb6cece10ec2db6c7

cd /tmp/issue8_eval/caatingarobotics && git rev-parse HEAD
# 840d18cf454a7c5cc911c6f0d93560519fbf19f7
```

## Core evaluation commands
EasyNavigation:
```bash
cd /tmp/issue8_eval/EasyNavigation
source /opt/ros/humble/setup.bash
rosdep check --from-paths . --ignore-src -r --rosdistro humble
rosdep check --from-paths . --ignore-src -r --rosdistro jazzy
colcon build --packages-select easynav_interfaces --event-handlers console_direct+
colcon build --packages-up-to easynav_tools --event-handlers console_direct+
source install/setup.bash
timeout 15 ros2 run easynav_tools tui --help
```

Topological Navigation:
```bash
cd /tmp/issue8_eval/topological_navigation_aoc_refactor
source /opt/ros/humble/setup.bash
rosdep check --from-paths . --ignore-src -r --rosdistro humble
rosdep check --from-paths . --ignore-src -r --rosdistro jazzy
colcon build --packages-up-to topological_navigation --event-handlers console_direct+
source install/setup.bash
timeout 15 python3 topological_navigation/topological_navigation/scripts/map_manager2.py --help
timeout 15 python3 topological_navigation/topological_navigation/scripts/map_manager2.py --test
```

Topological current node publication check:
```bash
cd /tmp/issue8_eval/topological_navigation_aoc_refactor
source /opt/ros/humble/setup.bash
source install/setup.bash
(timeout 15 python3 topological_navigation/topological_navigation/scripts/map_manager2.py --test > /tmp/issue8_eval/topnav_map_manager_for_loc.log 2>&1 &) 
sleep 2
(timeout 10 python3 topological_navigation/topological_navigation/scripts/localisation2.py > /tmp/issue8_eval/topnav_localisation2.log 2>&1 &)
sleep 3
ros2 topic list | sort | rg "current_node|closest_node|topological_map_2|topological_map_schema"
```

Caatinga:
```bash
cd /tmp/issue8_eval/caatingarobotics
source /opt/ros/humble/setup.bash
rosdep check --from-paths src --ignore-src -r --rosdistro humble
rosdep check --from-paths src --ignore-src -r --rosdistro jazzy
colcon build --base-paths src --packages-select caatinga_vision --event-handlers console_direct+
colcon build --base-paths src --packages-select agro_robot_sim --event-handlers console_direct+
source install/setup.bash
timeout 12 ros2 launch agro_robot_sim sim.launch.py
timeout 18 ros2 launch agro_robot_sim fazenda_completa.launch.py
```

## Outcome summary by command group
| Stack | Dependency audit | Minimal build | Runtime smoke | Notes |
|---|---|---|---|---|
| EasyNavigation | `rosdep` failed on `yaets` and Jazzy-on-jammy mappings | Build passed for `easynav_interfaces` and `easynav_tools` path | `tui` failed due missing `rich` module | Runtime friction mainly Python dependency provisioning |
| Topological Navigation | `rosdep` reported test dependency gaps only (`launch_pytest` on Jazzy/jammy mismatch) | Build passed for msgs and core package | `map_manager2 --test` loaded map successfully | `/current_node` and graph topics observed when localiser was launched |
| Caatinga | `rosdep` flagged `ament_python` key in this host | Build passed for `caatinga_vision` and `agro_robot_sim` | `sim.launch.py` and `fazenda_completa.launch.py` started; instability observed in full launch | Strong alignment, but launch hardening needed |

## Key log evidence
### EasyNavigation
```text
ERROR[easynav_common]: Cannot locate rosdep definition for [yaets]
...
ModuleNotFoundError: No module named 'rich'
```

### Topological Navigation
```text
[INFO] [topological_map_manager_2]: Map loaded and validated successfully.
[INFO] [topological_map_manager_2]: Map ready -- name=test_map, nodes=5
[INFO] [topological_localisation]: Graph built: 5 nodes, 4 edges
```

Observed topics during manager + localisation smoke:
```text
/closest_edges
/closest_node
/closest_node_distance
/current_node
/current_node/tag
/topological_map_2
/topological_map_schema
```

### Caatinga
Observed topics during `sim.launch.py` smoke:
```text
/clock
/cmd_vel
/joint_states
/odom
/rosout
/tf
/tf_static
```

Evidence of full launch activity and instability:
```text
[INFO] [launch.user]: GPS mode ativo: AMCL desativado, map_server ativo.
[ERROR] [gzserver-1]: process has died ... exit code 255 ...
[ERROR] [rviz2-18]: process has died ... exit code 127 ...
```

## Mitigation checklist for Caatinga recommendation
1. Stabilize `gzserver` startup in `fazenda_completa.launch.py`.
2. Make RViz optional in launch flow for non-desktop and CI paths.
3. Add startup health checks before lifecycle manager transition chain.
4. Repeat this runbook in Ubuntu 24.04 with ROS 2 Jazzy before final merge decision.

## GitHub delivery text
### Milestone request comment for Issue #8
```text
I prepared a full evidence-backed comparison for #8 and have a PR ready to open against `sowbot`.
The current PR workflow requires a milestone, but there are no milestones available in the repo right now.
Could a maintainer please create or assign a milestone for this PR path so CI can pass the milestone check?
```

### PR body draft (template-compliant)
```text
### Motivation

This PR closes #8 by adding a reproducible comparison of EasyNavigation, Topological Navigation (aoc_refactor), and Caatinga.
It prioritizes Day-0 operability and strategic alignment with Sowbot, with explicit spotlight and mitigation plan for Caatinga.
Closes #8.

### Implementation

- Added `docs/nav_stack_selection_issue_8.md` with:
  - objective and scope
  - mandatory `Caatinga Spotlight`
  - methodology, weighted matrix, and recommendation
  - mitigation and adoption plan
- Added `docs/evidence/issue_8_runbook.md` with:
  - exact commands
  - key logs
  - reproducibility notes and GitHub delivery snippets

### Progress

- [x] I chose a meaningful title that completes the sentence: "If applied, this PR will..."
- [x] I chose meaningful labels (if GitHub allows me to so).
- [x] The implementation is complete.
- [ ] Tests with a real hardware have been successful (or are not necessary).
- [x] Pytests have been added (or are not necessary).
- [x] Documentation has been added (or is not necessary).
```

### Issue closing comment draft
```text
Implemented in PR: <PR_LINK>

Summary:
- Completed reproducible Day-0 comparison across EasyNavigation, Topological Navigation, and Caatinga.
- Added explicit `Caatinga Spotlight` and weighted decision matrix.
- Recommendation: Caatinga with mitigation, while keeping Topological Navigation as task-layer candidate.
- Included command-level runbook and key logs for repeatability.

Main caveat:
- This pass ran on Ubuntu 22.04 + ROS 2 Humble.
- A final confirmation run on Ubuntu 24.04 + Jazzy is still required before production rollout.
```
