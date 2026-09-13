# Sowbot Safety Roadmap, v0.1

## 1. E-stop and safety hardware

| Item | Status | Notes |
|---|---|---|
| /estop/soft (software) | DONE | driver-level, devkit_driver |
| /estop/front, /estop/back (hardware state topics) | DONE | driver-level |
| Bumper topics (front_top, front_bottom, back) | DONE | real-time cutoff on ESP32 |
| Physical hard-wired E-stop | Work in progress | README states it is mandatory, not confirmed built or tested on current hardware |
| Bumper e-stops (physical) | TO DO | hardware bumper strips wired direct to E-stop circuit, separate from the software bumper topics above |
| Wireless failsafe pendant | TO DO | - E-stop: [Indus 1S transmitter](https://telemandosybaterias.com/en/p/indus-1s-868mhz-tyro-remotes-e-stop-wireless) — €725 + [Gemini 1S receiver](https://telemandosybaterias.com/en/p/gemini-230vac-tyro-remotes-e-stop-wireless) — €771 (both excl. VAT, excl. delivery) = **€1,496 for the pair** PL-c (EN ISO 13849-1) |
| First-run terminal acceptance of E-stop / safety warning & disclaimer| DONE | prompted by manage.py during .env setup |
| Resume confirmation after a stop | TO DO | no step requires a human to confirm before the robot resumes after bumper, e-stop, or sentor-triggered stop |

## 2. Monitoring

| Item | Status | Notes |
|---|---|---|
| sentor (sowbot_monitor.yaml) | Work in progress, ~75% | monitors e-stop, bumpers, battery, camera, odom, neo heartbeat |
| sentor_node.py wired into devkit.launch.py | DONE | |
| sentor hardware smoke test | TO DO | validated in sim only so far |
| Battery voltage cutoff threshold | TO DO | marked `# TODO: CONFIRM` in sowbot_monitor.yaml, no value set |
| ros2_medkit black-box logging | TO DO | not in repo or dependency list yet, adding ASAP |

## 3. Perception

| Item | Status | Notes |
|---|---|---|
| Thermal (MLX90640/MLX90614) human detection | TO DO | not in repo or BOM |
| Livestock false-positive tolerance | AGREED | fine for thermal to stop on livestock, safe default |

## 4. Controller path

| Item | Status | Notes |
|---|---|---|
| ESP32 + Lizard DSL | DONE, current | hard real-time motor PID and bumper cutoff |
| STM32H7 + Ardurover migration | TO DO | EKF3, failsafes, geofencing, SITL testing, community scrutiny. In time should buy IEC 61508 REF: [ArduPilot Zephyr HAL: Flying on a BeagleV-Fire](https://www.beagleboard.org/projects/ardupilot-on-zephyr-flying-on-the-beaglev-fire) · [Zephyr Safety Overview](https://docs.zephyrproject.org/latest/safety/safety_overview.html)  |

## 5. Regulatory compliance roadmap

No compliance claimed. Reference standards only until formal assessment or audit is done.

| Standard | Domain | Relevance |
|---|---|---|
| ISO 18497 | Highly automated ag machinery | Hazard zones, operational modes for autonomous field work |
| ISO 25119 / AgPL | Tractor and ag electronics functional safety | AgPL target for motor-stop interlocks |
| ISO 13849 / PL | Machinery safety, control systems | PL target for E-stop relay, bumper circuit |
| ISO 3691-4 | AGV obstacle detection | Clearance rules, braking distance, detection envelope sizing |
| IEC 61508 | Functional safety, E/E/PE systems | Reference for controller firmware architecture |
| ISO 21448 (SOTIF) | Safety of the intended functionality | Vision degradation: mud, dust, glare |

### Phase 1: dev platform (current focus)

Audience: university labs, ag-tech researchers, software startups.

| Item | Status |
|---|---|
| Formal PL/AgPL calculation | NOT DONE |
| Third-party audit | NOT DONE |
| Compliance claim in docs or marketing | NONE, correctly |
| Standards used as design reference | YES, informal |
| Liability position | user's own risk, standard for a research/dev kit, stated in README |

No certification work needed at this phase. Reference the standards, do not claim them.

### Phase 2: OEM modular subsystems

Audience: existing ag equipment manufacturers integrating Sowbot's drive/safety core.

| Item | Status |
|---|---|
| PL/AgPL calculation for E-stop and bumper circuit | TO DO, gate for this phase |
| SOTIF assessment for vision degradation cases | TO DO |
| IEC 61508 architecture review | TO DO |
| Third-party certification | NOT REQUIRED YET, formal internal assessment is |

### Phase 3: commercial sale to farmers

Audience: commercial growers, farm management enterprises.

| Item | Status |
|---|---|
| Full ISO 18497 compliance | REQUIRED before sale |
| ISO 13849 PLd certification, or equivalent | REQUIRED before sale |
| Third-party safety audit | REQUIRED before sale |
| Field trial history | REQUIRED before sale |
| Insurance and liability structure | NOT YET SCOPED |


## Build order

1. Confirm battery cutoff threshold
2. Build and test physical hard-wired E-stop and bumper e-stops on current hardware
3. Add wireless failsafe pendant
4. sentor hardware smoke test
5. ros2_medkit black-box logging
6. Decide on resume-confirmation behaviour

