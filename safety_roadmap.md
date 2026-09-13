# Sowbot Safety Roadmap, v0.1

## 1. E-stop and safety hardware

| Item | Status | Notes |
|---|---|---|
| /estop/soft (software) | DONE | driver-level, devkit_driver |
| /estop/front, /estop/back (hardware state topics) | DONE | driver-level |
| Bumper topics (front_top, front_bottom, back) | DONE | real-time cutoff on ESP32 |
| Physical hard-wired E-stop | Work in progress | README states it is mandatory, not confirmed built or tested on current hardware |
| Bumper e-stops (physical) | TO DO | Tapeswitch selected, 2-wire configuration, PRSU/2 controller. See sourcing and calculation notes below |
| Wireless failsafe pendant | TO DO | [Indus 1S transmitter](https://telemandosybaterias.com/en/p/indus-1s-868mhz-tyro-remotes-e-stop-wireless), €725, plus [Gemini 1S receiver](https://telemandosybaterias.com/en/p/gemini-230vac-tyro-remotes-e-stop-wireless), €771 (both excl. VAT, excl. delivery), €1,496 for the pair. Manufacturer claims PL-c, not yet checked against their declaration of conformity |
| Reversing alarm and flashing LED, motion-active | TO DO | [Brigade self-adjusting white sound reversing alarm](https://brigade-electronics.com/warning-systems/reversing-and-warning-alarms/self-adjusting-white-sound-reversing-alarms/), ambient-adjusting, integrated LED. Wire to motion state generally, not just reverse. Confirm 12/24V compatibility with the current battery bus. This supports the avoidance assumption in the risk graph below, it is not part of the stop function itself |
| First-run terminal acceptance of E-stop / safety warning and disclaimer | DONE | prompted by manage.py during .env setup |
| Resume confirmation after a stop | TO DO | no step requires a human to confirm before the robot resumes after a bumper, e-stop, or sentor-triggered stop |

### Bumper switch sourcing

Proper industrial safety bumpers with published PL ratings:

| Supplier | Model | Notes |
|---|---|---|
| [Unchained Robotics](https://unchainedrobotics.de/en/brands/aso-safety-solutions) | ASO SENTIR | PLc-d from €860 |
| [Tapeswitch Corp.](https://www.tapeswitch.com/bumpers.html) | VBL, SE-45D, SE-75D, custom SE-C series | **Selected.** 2-wire configuration with PRSU/2 controller, Category 1. Datasheets and quote request forms |
| [ABB Safety](https://new.abb.com/low-voltage/products/safety-products/pressure-sensitive-devices/asb) | ASB safety bumper | CAD drawings, 2D/3D data, custom foam or leather lengths 0.2m to 3.0m |
| [Schmersal](https://products.schmersal.com/en_US/safety-related-bumper-1000074843) | SSG-SBL | Dual-channel, heavy-duty, technical specs and contact form |
| [Mayser](https://www.mayser.com/en/safety-technology/products/safety-bumpers) | Custom safety bumpers | Optoelectronic and polyurethane foam options, configuration portal |

## 2. Monitoring

| Item | Status | Notes |
|---|---|---|
| sentor (sowbot_monitor.yaml) | Work in progress, ~75% | monitors e-stop, bumpers, battery, camera, odom, neo heartbeat |
| sentor_node.py wired into devkit.launch.py | DONE | |
| sentor hardware smoke test | TO DO | validated in sim only so far |
| Battery voltage cutoff threshold | TO DO | marked `# TODO: CONFIRM` in sowbot_monitor.yaml, no value set |
| ros2_medkit black-box logging | TO DO | not in repo or dependency list yet, adding ASAP |

Note: sentor and the software e-stop topics are diagnostic and supervisory. They do not count as safety-related parts of the control system for the PLc calculation below.

## 3. Perception

| Item | Status | Notes |
|---|---|---|
| Thermal (MLX90640/MLX90614) human detection | TO DO | not in repo or BOM |
| Livestock false-positive tolerance | AGREED | fine for thermal to stop on livestock, safe default |

## 4. Controller path

| Item | Status | Notes |
|---|---|---|
| ESP32 + Lizard DSL | DONE, current | hard real-time motor PID and bumper cutoff |
| STM32H7 + Ardurover migration | TO DO | EKF3, failsafes, geofencing, SITL testing, community scrutiny. In time should buy IEC 61508. Reference: [ArduPilot Zephyr HAL: Flying on a BeagleV-Fire](https://www.beagleboard.org/projects/ardupilot-on-zephyr-flying-on-the-beaglev-fire), [Zephyr Safety Overview](https://docs.zephyrproject.org/latest/safety/safety_overview.html) |

## 5. Regulatory compliance

No compliance claimed. Reference standards only until formal assessment or audit is done.

**Target: PLc, ISO 13849-1, for the whole-vehicle stop function.**

| Standard | Domain | Relevance |
|---|---|---|
| ISO 18497 | Highly automated ag machinery | Hazard zones, operational modes for autonomous field work |
| ISO 25119 / AgPL | Tractor and ag electronics functional safety | AgPL target for motor-stop interlocks |
| ISO 13849 / PL | Machinery safety, control systems | PLc target for E-stop relay, bumper circuit |
| ISO 3691-4 | AGV obstacle detection | Clearance rules, braking distance, detection envelope sizing |
| IEC 61508 | Functional safety, E/E/PE systems | Reference for controller firmware architecture |
| ISO 21448 (SOTIF) | Safety of the intended functionality | Vision degradation: mud, dust, glare |

### PLc calculation, draft

**Risk graph:** S2, F2, P1. Gives PLc. Re-check P if row spacing or travel speed cut down a person's chance to get clear, that would push the target to PLd.

**In scope for the calculation:** hard-wired E-stop, physical bumper switch, wireless failsafe pendant.

**Out of scope:** software e-stop topics, sentor monitoring, reversing alarm and LED. None of these are safety-related parts of the control system.

**Architecture:** Category 1, single channel, no diagnostic coverage. Ceiling is PLc provided MTTFd is in the high band.

**Bumper and E-stop, Tapeswitch PRSU/2, 2-wire configuration:** confirmed by Tapeswitch as Category 1, PLd unreachable in this configuration, which matches the PLc target. MTTFd and PFHd for this specific configuration not yet obtained, only the 4-wire dual-channel figures are published and those do not apply here. Needed from Tapeswitch directly.

**Wireless pendant, Indus 1S / Gemini 1S:** manufacturer claims PL-c, not yet checked against their declaration of conformity.

**Category 1 requirements, clause 6.2.4:** well-tried component status for the switch and E-stop, MTTFd in the high band. No CCF requirement, no diagnostic coverage requirement.

**Annex F:** does not apply. Annex F is CCF scoring for multi-channel architectures, Category 2 to 4. Category 1 is single channel, so there is no second channel for a common cause to act on.

**Combination rule:** E-stop, bumper, and pendant form a series safety function. The lowest PL of the three sets the ceiling for the whole function, not an average.

**Outstanding before this is a finished calculation:**
1. MTTFd/PFHd for the Tapeswitch 2-wire configuration, from Tapeswitch
2. Well-tried component justification for the switch and E-stop, documented
3. Pendant PL-c claim checked against its declaration of conformity
4. Numbers run through SISTEMA or equivalent once the above three are in

Until then this is a target and a chosen architecture, not a calculated figure.

### Phase 1: dev platform (current focus)

Audience: university labs, ag-tech researchers, software startups.

| Item | Status |
|---|---|
| PLc calculation | Draft in progress, see above |
| Third-party audit | NOT DONE |
| Compliance claim in docs or marketing | NONE, correctly |
| Standards used as design reference | YES, informal |
| Liability position | user's own risk, standard for a research/dev kit, stated in README |

No certification work needed at this phase. Reference the standards, do not claim them.

### Phase 2: OEM modular subsystems

Audience: startups integrating Sowbot's drive/safety core.

| Item | Status |
|---|---|
| PLc calculation for E-stop and bumper circuit | Draft in progress, gate for this phase |
| SOTIF assessment for vision degradation cases | TO DO |
| IEC 61508 architecture review | TO DO |
| Third-party certification | NOT REQUIRED YET, formal internal assessment is |

### Phase 3: commercial sale to farmers

Audience: commercial growers, farm management enterprises.

| Item | Status |
|---|---|
| Full ISO 18497 compliance | REQUIRED before sale |
| ISO 13849 PLc certification, or equivalent | REQUIRED before sale |
| Third-party safety audit | REQUIRED before sale |
| Field trial history | REQUIRED before sale |
| Insurance and liability structure | NOT YET SCOPED |

## Build order

1. Confirm battery cutoff threshold
2. Get MTTFd/PFHd figures from Tapeswitch for the 2-wire configuration
3. Build and test physical hard-wired E-stop and bumper e-stops on current hardware
4. Add wireless failsafe pendant, check PL-c claim against declaration of conformity
5. Add reversing alarm and flashing LED, wired to motion state
6. Document well-tried component justification and finish the PLc calculation
7. sentor hardware smoke test
8. ros2_medkit black-box logging
9. Decide on resume-confirmation behaviour
