# Sowbot Safety Roadmap, v0.2

## 1. E-stop and safety hardware+24V Safety Power
```
======================= 24V SAFETY CONTROL LOOP =======================

+24V Safety Power
       │
┌──────┴──────────────────────────────────────┐
│  Physical E-Stop Buttons                    │  (Schneider XALK178, 2x NC Contacts)
└──────┬──────────────────────────────────────┘
       │
┌──────┴──────────────────────────────────────┐
│  Gemini 1S Receiver                         │  (Tyro Wireless E-Stop Contacts)
└──────┬──────────────────────────────────────┘
       │
┌──────┴──────────────────────────────────────┐
│  PRSU/2 Controller                          │  (Tapeswitch VBL Bumper Contacts)
└──────┬──────────────────────────────────────┘
       ├──────────────────────┐
       │                      │
┌──────┴───────────┐   ┌──────┴───────────┐
│ SW180 #1 Coil    │   │ SW180 #2 Coil    │  (24V DC Actuation Coils)
│ [TVS Suppressor] │   │ [TVS Suppressor] │
└──────┬───────────┘   └──────┬───────────┘
       │                      │
───────┴──────────────────────┴──────────────────────────────── GND (24V)


======================= 48V TRACTION POWER BUS =======================

+48V Battery (B+)
       │
 ┌─────┴──────────────┐
 │ SW180 #1 Contacts  │  (First Isolation Break)
 └─────┬──────────────┘
       │  (48V Series Link)
 ┌─────┴──────────────┐
 │ SW180 #2 Contacts  │  (Second Isolation Break)
 └─────┬──────────────┘
       │
  Motor Controllers 
```

| Item | Status | Notes |
|---|---|---|
| /estop/soft (software) | DONE | driver-level, devkit_driver |
| /estop/front, /estop/back (hardware state topics) | DONE | driver-level |
| Bumper topics (front_top, front_bottom, back) | DONE | real-time cutoff on ESP32 |
| Physical hard-wired E-stop | Work in progress | README states it is mandatory, not confirmed built or tested on current hardware |
| Bumper e-stops (physical) | TO DO | Tapeswitch selected, 2-wire configuration, PRSU/2 controller. See sourcing and calculation notes below |
| E-stop/bumper output stage (power cutoff) | TO DO | PRSU/2 output relays cannot switch the 48V/40A motor bus directly. See contactor sourcing below |
| Wireless failsafe pendant | TO DO | [Indus 1S transmitter](https://telemandosybaterias.com/en/p/indus-1s-868mhz-tyro-remotes-e-stop-wireless), €725, plus [Gemini 1S receiver](https://telemandosybaterias.com/en/p/gemini-230vac-tyro-remotes-e-stop-wireless), €771 (both excl. VAT, excl. delivery), €1,496 for the pair. Manufacturer claims PL-c, not yet checked against their declaration of conformity |
| Reversing alarm and flashing LED, motion-active | TO DO | [Brigade self-adjusting white sound reversing alarm](https://brigade-electronics.com/warning-systems/reversing-and-warning-alarms/self-adjusting-white-sound-reversing-alarms/), ambient-adjusting, integrated LED. Wire to motion state generally, not just reverse. Confirm 12/24V compatibility with the current battery bus. This supports the avoidance assumption in the risk graph below, it is not part of the stop function itself |
| First-run terminal acceptance of E-stop / safety warning and disclaimer | DONE | prompted by manage.py during .env setup |
| Resume confirmation after a stop | TO DO | no step requires a human to confirm before the robot resumes after a bumper, e-stop, or sentor-triggered stop |

### Bumper switch sourcing


| Supplier | Model | Notes |
|---|---|---|
| [Tapeswitch Corp.](https://www.tapeswitch.com/bumpers.html) | VBL, SE-45D, SE-75D, custom SE-C series | **Selected.** 2-wire configuration with PRSU/2 controller, Category 1. Datasheets and quote request forms |

### PRSU/2 controller, output rating

[PRSU/2](https://www.tapeswitch.com/controllers/prsu2.html) device rating: Category 3, PL-e, TÜV-assessed, response < 30ms. Output stage: 2 N.O. positive-guided safety relays, AgSnO2 contacts, switching voltage 250VAC or 24VDC, max switch/relay current 6A individual, 13.8A combined.

| Part | Coil | Rating | Notes |
|---|---|---|---|
| Albright SW180, 24V coil variant | 24VDC | 200A continuous, 400A peak, magnetic blowout, silver alloy contacts | Matches PRSU/2 output rating directly, no interposing relay needed. ~$70-130/unit. No aux contact by default, order aux-contact variant or add-on kit separately |


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
| STEVAL-SILPLC01 evaluated as candidate STM32H7 board | EVALUATED, not adopted for safety role | Hardware TÜV Italia-assessed SIL2/PL-d (1oo2, STM32H723VG, X-CUBE-STL self-test library certified by TÜV Rheinland). Running ArduPilot on it invalidates that assessment, ArduPilot has no IEC 61508/ISO 13849 systematic capability evidence and was not developed under a safety lifecycle. Board can be used for non-safety motion control, cannot be counted as part of the safety-related control system as currently planned |
| EtherCAT link, STEVAL-SILPLC01 to motor drivers and Avaota A1, via [SG Electronic Systems EtherCAT shield](https://www.sg-electronic-systems.com/ecommerce/ethernet-shield/37-etherc-v163-ethercatr-is-an-ethernet-based-fieldbus-system-invented-by-beckhoff-automation-the-protocol-is-standardized-in-iec-6.html) | EVALUATED, not safety-rated | Standard EtherCAT, not FSoE (Safety over EtherCAT, ETG.5100). Plain EtherCAT carries no safety semantics, this link cannot carry safety-related stop/interlock data without an FSoE-certified master and slave stack, which this shield does not provide |

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

**In scope for the calculation:** hard-wired E-stop, physical bumper switch, wireless failsafe pendant, output stage contactor.

**Out of scope:** software e-stop topics, sentor monitoring, reversing alarm and LED, ArduPilot/STEVAL-SILPLC01 motion control path, EtherCAT link to motor drivers and Avaota A1. None of these are safety-related parts of the control system as currently architected.

**Architecture:** Category 1, single channel, no diagnostic coverage. Ceiling is PLc provided MTTFd is in the high band.

**Bumper and E-stop, Tapeswitch PRSU/2, 2-wire configuration:** confirmed by Tapeswitch as Category 1, PLd unreachable in this configuration, which matches the PLc target. MTTFd and PFHd for this specific configuration not yet obtained, only the 4-wire dual-channel figures are published and those do not apply here. Needed from Tapeswitch directly.

**Output stage contactor:** not yet selected, see contactor sourcing above. Under the Category 1 architecture, a single well-tried contactor with adequate current/voltage margin satisfies clause 6.2.4, no CCF or diagnostic coverage requirement. Aux-contact cross-monitoring and a second parallel contactor are a Category 3-style upgrade, not required for PLc, would only be relevant if the target changes to PLd/e in Phase 3.

**Wireless pendant, Indus 1S / Gemini 1S:** manufacturer claims PL-c, not yet checked against their declaration of conformity.

**Category 1 requirements, clause 6.2.4:** well-tried component status for the switch, E-stop, and contactor, MTTFd in the high band. No CCF requirement, no diagnostic coverage requirement.

**Annex F:** does not apply. Annex F is CCF scoring for multi-channel architectures, Category 2 to 4. Category 1 is single channel, so there is no second channel for a common cause to act on.

**Combination rule:** E-stop, bumper, pendant, and output contactor form a series safety function. The lowest PL of these sets the ceiling for the whole function, not an average.

**Outstanding before this is a finished calculation:**
1. MTTFd/PFHd for the Tapeswitch 2-wire configuration, from Tapeswitch
2. Well-tried component justification for the switch and E-stop, documented
3. Contactor selected (coil voltage matched to PRSU/2 output, current/voltage margin confirmed), well-tried component justification documented
4. Pendant PL-c claim checked against its declaration of conformity
5. Numbers run through SISTEMA or equivalent once the above four are in

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
3. Select output stage contactor, resolve coil voltage against PRSU/2 output rating, decide single vs redundant contactor
4. Build and test physical hard-wired E-stop and bumper e-stops on current hardware
5. Add wireless failsafe pendant, check PL-c claim against declaration of conformity
6. Add reversing alarm and flashing LED, wired to motion state
7. Document well-tried component justification and finish the PLc calculation
8. sentor hardware smoke test
9. ros2_medkit black-box logging
10. Decide on resume-confirmation behaviour
