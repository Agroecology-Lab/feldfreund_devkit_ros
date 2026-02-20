# Issue #8: Navigation Stack Selection with Caatinga Spotlight

## Objective and scope
This report addresses [Issue #8](https://github.com/Agroecology-Lab/feldfreund_devkit_ros/issues/8).
The objective is a reproducible Day-0 comparison of EasyNavigation, Topological Navigation, and Caatinga.
The decision emphasis is strategic alignment with the Sowbot roadmap while preserving technical traceability.
The scope is analysis and recommendation only.
No runtime API was changed in `feldfreund_devkit_ros`.

## Caatinga Spotlight
Caatinga is the strongest fit for the navigation middle layer in the current Sowbot direction.
The stack already exposes launch assets for `sim.launch.py` and `fazenda_completa.launch.py` with Nav2 integration points.
The package set is small and understandable for first-contact onboarding.
The simulation launch proved functional enough to publish core motion topics in this host test.
The current gaps are concentrated in launch robustness and environment compatibility rather than architectural mismatch.

### Role in the stack
Caatinga is treated as the navigation execution layer for pose-to-pose movement.
Topological Navigation remains a strong task-layer candidate for semantic mission logic.
This keeps the intended layered architecture from issue discussion on 2026-02-20.

### Agricultural fit
Caatinga brings direct alignment with outdoor farm workflows already referenced by maintainers.
Its launch structure includes farm-specific worlds, maps, and mission entry points.
This reduces translation cost from roadmap intent to runnable artifacts.

### Known gaps and required mitigations
`fazenda_completa.launch.py` starts key Nav2 nodes, but `gzserver` instability was observed in this environment.
`rviz2` failed with a GLIBC symbol lookup error in this host, which cascaded lifecycle noise.
Mitigations are listed in the final recommendation plan and must be completed before production use.

## Methodology and test environment
Evaluation date was 2026-02-20 UTC.
Host environment was Ubuntu 22.04.5 with ROS 2 Humble and Python 3.10.
Issue target environment is Ubuntu 24.04 with ROS 2 Jazzy.
Because of this mismatch, results are a Day-0 proxy and not a final Jazzy sign-off.

Candidate references:

| Candidate | Reference | Commit used |
|---|---|---|
| EasyNavigation | https://github.com/EasyNavigation/EasyNavigation | `ce3fedba579f9bb24bf3fb59feb51e92e629af4d` |
| Topological Navigation | https://github.com/LCAS/topological_navigation/tree/aoc_refactor | `cc9869223fa4bf568881e2dfb6cece10ec2db6c7` |
| Caatinga | https://github.com/samuk/caatingarobotics | `840d18cf454a7c5cc911c6f0d93560519fbf19f7` |

Evaluation steps were:
1. `rosdep check` for host distro and Jazzy target proxy.
2. Minimal `colcon build` for representative packages.
3. Runtime smoke commands with `timeout` to verify process startup and topic availability.
4. Log capture and weighted scoring.

## Results by candidate

### EasyNavigation
Package discovery found 12 packages.
`rosdep` reported unresolved keys in this host for `yaets` and Jazzy-on-jammy definitions.
Minimal builds succeeded for `easynav_interfaces` and the `easynav_tools` path.
Runtime smoke for `ros2 run easynav_tools tui --help` failed with `ModuleNotFoundError: No module named 'rich'`.
TUI capability exists and is discoverable in package metadata.
Code references confirm support for both `Twist` and `TwistStamped` pathways.

### Topological Navigation (aoc_refactor)
Package discovery found 3 packages.
Package metadata confirms `ament_python` build type for `topological_navigation`.
No `mongodb_store` dependency was found in active package manifests or runtime smoke logs.
Minimal builds for `topological_navigation_msgs` and `topological_navigation` succeeded.
`map_manager2.py --test` loaded `test_simple_tmap2.yaml` and reported map-ready status.
With `localisation2.py` running, topics `/current_node`, `/closest_node`, and `/topological_map_2` were visible.
This validates stateless graph serving and current-node publication behavior without DB coupling.

### Caatinga
Package discovery found 2 packages under `src`.
Minimal builds succeeded for `caatinga_vision` and `agro_robot_sim`.
`sim.launch.py` started Gazebo and robot spawn, then published `/cmd_vel`, `/odom`, `/joint_states`, `/tf`, and `/clock`.
`fazenda_completa.launch.py` started Nav2-related processes but showed `gzserver` exit 255 in this host run.
`rviz2` failed with GLIBC symbol lookup error and triggered lifecycle churn after signal handling.
Even with those issues, the launch path demonstrates direct integration direction with Nav2 and GPS localization nodes.

## Weighted decision matrix
Weights were locked to the requested policy:
1. Alignment with Sowbot and Caatinga strategy: 35%.
2. Day-0 friction: 30%.
3. Runtime minimum without DB dependency: 20%.
4. Compatibility with Jazzy and dependency surface: 15%.

Scoring is from 0 to 10.
Weighted total equals the sum of `score * weight`.

| Criterion | Weight | EasyNavigation | Topological Navigation | Caatinga |
|---|---:|---:|---:|---:|
| Alignment with Sowbot/Caatinga | 0.35 | 6.0 | 8.0 | 10.0 |
| Day-0 friction | 0.30 | 6.0 | 7.0 | 6.5 |
| Runtime minimum without DB | 0.20 | 4.5 | 8.0 | 7.5 |
| Jazzy/dependency compatibility | 0.15 | 5.0 | 6.0 | 6.5 |
| **Weighted total** |  | **5.55** | **7.40** | **8.00** |

## Decision and recommendation
Primary recommendation is **Caatinga with mitigation**.
Caatinga is the top weighted result and the strongest roadmap alignment.
Topological Navigation should be retained as the semantic task-layer candidate to pair with Caatinga navigation execution.
EasyNavigation remains useful for quick tooling patterns but is not the primary recommendation for this issue outcome.

## Mitigation plan required for Caatinga
1. Stabilize Gazebo startup in `fazenda_completa.launch.py`.
   Current bug: `gzserver` exit 255 on world startup in this host run.
   Workaround: use `sim.launch.py` for smoke and run headless validation before full mission launch.
   Next milestone: stable 5-minute bringup with no `gzserver` crash.
2. Remove RViz host linker fragility from Day-0 path.
   Current bug: `rviz2` GLIBC symbol lookup failure.
   Workaround: run without RViz in automated validation, or force a non-snap runtime environment.
   Next milestone: optional RViz launch argument with default disabled for CI.
3. Harden lifecycle startup and shutdown flow.
   Current bug: lifecycle noise after upstream process failures and forced interrupts.
   Workaround: keep smoke timeout bounded and avoid treating interrupted teardown as functional failure.
   Next milestone: deterministic startup health checks before enabling mission loop.

## Adoption plan
### Short term (0-2 weeks)
Use Caatinga as the navigation baseline in simulation workflows.
Keep Topological Navigation in parallel smoke validation for task-layer integration.
Add Ubuntu 24.04 and ROS 2 Jazzy validation pass with the same runbook commands.

### Medium term (3-6 weeks)
Wire Topological Navigation mission intents into Caatinga navigation endpoints.
Standardize launch profiles for `sim`, `gps-nav`, and `task-graph` modes.
Promote a single documented Day-0 script for contributor onboarding.

## Limits and follow-up
This report was executed on Ubuntu 22.04 with ROS 2 Humble, not the target Ubuntu 24.04 Jazzy.
A final go/no-go should be gated by one dedicated rerun in the target environment.
The rerun should reuse the same command protocol in `docs/evidence/issue_8_runbook.md`.
