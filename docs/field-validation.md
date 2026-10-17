# Field validation progression

Status: proposed, not yet agreed. Tracked as O42 in [safety_roadmap.md](../safety_roadmap.md).
Numbers marked *proposed* are starting points to argue about, not requirements.

Simulation does not count towards any step below: Gazebo does not model track slip, so sim results say little about the tracked hardware.

## Rules for every step

- Move to the next step only when every exit criterion of the current one is met and written down.
- Each run records the date, commit hash, parameters (`cmd_vel_timeout_s`, `GUID_TIMEOUT`, Nav2 speed limits) and the metrics below.
- Before any operation, the hard-wired 24V loop, including the physical E-stop path, must be built and tested (safety_roadmap.md O7), or a separately validated independent stop method must be available; otherwise operation is prohibited.
- Until the rated stop function is complete (safety_roadmap.md O7, O8, O14), every permitted step is supervised-only: a competent person within reach of a physical E-stop (or the separately validated stop control), nobody else in the working area, and no safety claim made.
- Set `SOWBOT_UI_PASSWORD` and keep the robot's network private for any run with the dashboard enabled.

## Metrics to record

| Metric | Definition |
|---|---|
| Cross-track error | RMS and maximum, per row pass, against the planned line |
| RTK fixed share | Fraction of run time with `/gnss/fix` status 2, and time spent in float or single |
| Interventions per hour | Every human take-over, soft or hard stop, per hour of autonomous running |
| Stopping distance | Metres from the stop event to standstill, at 0.5 m/s and 1.6 m/s, for each stop source below |
| Command-loss latency | Time from the last `cmd_vel` to the controller commanding zero |
| Heading validity | Fraction of time `/gnss/heading` is published and below the variance threshold |

## Steps

### 1. Bench (wheels off the ground)

Purpose: confirm the control chain without the vehicle moving.

- Stop sources checked: physical E-stop, bumpers, radar trip, soft E-stop, `cmd_vel` loss, bridge loss, dashboard disconnect.
- Confirm the SW180 contacts open and the motor controllers behave on each stop, including battery disconnect (O36).
- Confirm ArduPilot GUIDED responds, `GPS1_TYPE=14` is set and the EKF origin behaves on boot (O22 d).

Exit criteria (proposed): every stop source stops the drive on the bench, ten times in a row; command-loss latency measured and below the value the stopping-distance budget needs.

### 2. Hard standing (vehicle on the ground, no crop)

Purpose: measure stopping distance and basic tracking on a surface where slip is small.

- Stopping distance for every stop source at 0.5 and 1.6 m/s (O33, O20).
- Straight-line and turn-on-the-spot tracking; check lever arms and odometry covariance against measured behaviour (O37).
- Behaviour on RTK float and single: pull the correction source and watch what the robot does (O38).

Exit criteria (proposed): stopping distances written down for every source and inside the detection and boundary margins used in O20 and O29; the degraded-GNSS response is defined and observed.

### 3. Empty field

Purpose: real ground, slip and GNSS conditions, with no crop to damage.

- Cross-track error over several row passes at working speed.
- Row-end turns, wet and dry ground if possible, GNSS under canopy edges.
- Geofence behaviour: boundary approach across the speed range (O22 b). Operate inside a physical boundary or under direct supervision, the geofence is not a safety function.

Exit criteria (proposed): metrics recorded for at least several hours of running; no unexplained intervention; the fence breach action confirmed to be a stop or hold (O22 a).

### 4. Supervised crop rows

Purpose: working conditions with crop present.

- Cross-track error and interventions per hour in the crop, with `sowbot_row_follow` row-loss handling exercised.
- Radar and bumper validation on the vehicle using manufacturer-approved surrogate targets in the test positions, per the manufacturer procedure (O14).
- Tests involving people are excluded from this stage and reserved for a separately controlled procedure after the stop function is complete.

Exit criteria (proposed): the field metrics are published in the README Field results section; the open items blocking Phase 1 are closed or explicitly accepted in writing.
