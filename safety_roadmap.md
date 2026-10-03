# Sowbot Safety Roadmap, v0.6.9
## Contents

- [0. Safety scoping](#0-safety-scoping)
- [1. Hazard identification](#1-hazard-identification)
- [2. E-stop and safety hardware](#2-e-stop-and-safety-hardware)
  - [24V SAFETY CONTROL LOOP](#24v-safety-control-loop)
  - [48V TRACTION POWER BUS](#48v-traction-power-bus)
  - [Core - PLd](#core---pld)
  - [Supplemental - outside the rated function](#supplemental---outside-the-rated-function)
  - [Planned - independent geofence](#planned---independent-geofence-not-in-current-core)
- [3. Monitoring & Supervisory](#3-monitoring--supervisory)
- [4. Perception](#4-perception)
- [5. Controller path](#5-controller-path)
  - [Independent geofence (planned)](#independent-geofence-planned)
- [6. Regulatory compliance](#6-regulatory-compliance)
  - [Product classification](#product-classification)
  - [Functional safety calculation (draft)](#functional-safety-calculation-draft)
- [7. Non-traction actuators (OPEN, scope flag)](#7-non-traction-actuators-open-scope-flag)
- [8. Open items register](#8-open-items-register)
- [9. Phased rollout](#9-phased-rollout)
  - [Phase 1: dev platform (current focus)](#phase-1-dev-platform-current-focus)
  - [Phase 2: OEM modular subsystems](#phase-2-oem-modular-subsystems)
  - [Phase 3: commercial sale to farmers](#phase-3-commercial-sale-to-farmers)

**Document control**
- Status: living document, Phase 1 (dev platform)
- Scope: whole-vehicle emergency motor-stop function, ISO 13849-1. Does not cover implement, PTO, or manipulator safety, see §7.
- Owner: TBD (assign)
- v0.6.9 changes: controller path set to ESP32 (now) then ArduPilot on LEVIA-H7 (§5); independent dual-channel geofence on 2× STEVAL-SILPLC01 added as planned (§0, §1 H5, §2, §5, §6, O24 to O32); cerebri and FRDM-A-S32K358 not selected (§5, O23).

---

## 0. Safety scoping

The formal safety function is the hardwired 24V E-stop loop (§2, Core), comprising the physical E-stops, the ASO Sentir bumpers, the human-detection radar (Inxpect S101A ×3 + C203A), and the 48V traction interlock (Albright SW180 ×2) they drive.

Everything else in this document (ROS 2 nodes, `sentor`, controller firmware, non-radar perception, geofencing, reversing alarm/beacon) is supervisory or defence-in-depth, **not** part of the rated safety function, and does not enter the PLd calculation in §6. The motion alarm/beacon supports the P1 risk-graph parameter (§6) but is not part of the stop function.

Controller hardware and firmware choice (ESP32/Lizard, ArduPilot on LEVIA-H7) does not move the safety boundary.

Planned change: an independent dual-channel geofence (2× STEVAL-SILPLC01, §2 Planned, §5) is intended as a separate safety function acting on the 48V isolation. It is not built, not assessed, and not in the §6 calculation. Until O25 is closed, all geofencing remains supervisory.

---

## 1. Hazard identification

Supports the S2/F2/P1 risk graph parameters in §6. Not exhaustive, add rows as identified.

| ID | Hazard | Cause | Exposure | Current mitigation | Residual risk |
|---|---|---|---|---|---|
| H1 | Crush/impact from moving vehicle | Software fault, sensor failure, operator error | Continuous during field operation (F2) | Physical bumper + E-stop loop + radar (Core, per §0); motion alarm/beacon (Supplemental) supports P1 | E-stop loop response <30ms; radar detection response <100ms (catalogue figure, not S101A-specific, confirm under O14); combined detection-to-stop latency ~100-130ms, not yet sized against ISO 3691-4 (O20) |
| H2 | Rollaway after stop | Stop on slope, no parking brake | Not yet assessed (CONFIRM operating slope range) | Worm gear drive (assumed 40:1) self-locks tracks when unpowered | OPEN. Self-locking ratio assumed, not confirmed against gearbox datasheet; no positive parking brake in BOM as backup (O13) |
| H3 | Undetected E-stop hardware degradation | Contactor welding, relay failure over time | Continuous | SW180 aux microswitches for external device monitoring (EDM) of the two series contactors | Covered by DCavg in §6, pending confirmation of the C203A feedback input (O6) and final CCF/SISTEMA figures |
| H4 | False negative on human detection | Radar coverage gaps; sensor failure | Continuous once deployed near people | Inxpect S101A ×3 + C203A control unit (Core, in-scope per §0) | OPEN. Mounting geometry for the 3-sensor layout not defined (O20). Moving-vehicle suitability confirmed by manufacturer instruction manual; mobile-application validation procedure still to be run on-vehicle (O14). Min. set distance (1 m) is close to the worst-case detection-envelope sizing from H1 |
| H5 | Boundary excursion (uncontained operation) | GNSS/positioning fault or degradation (multipath, jamming, blackout, heading unobservability), FusionCore/ArduPilot fence misconfiguration, EKF-origin fault on boot | Continuous once deployed autonomously outside a physically fenced area | ArduPilot Rover GUIDED-mode geofence, fed by FusionCore fused pose via `GPS_INPUT` (see §5); supervisory only, not in the PLd chain per §0. Planned: independent dual-channel geofence (2× STEVAL-SILPLC01, own RTK input per channel) acting on the 48V isolation (§5, O24 to O29); separate safety function, PLr not yet determined (O25) | OPEN. Governed by ISO 18497-3 (design reference, see §6); requires ISO 12100 hazard treatment and ISO 18497-4 verification (worst-case speed, GNSS-degradation scenarios) before Phase 3, see O22. FusionCore's documented limits: yaw unobservable from IMU+encoder+GPS alone without magnetometer/dual-antenna heading; long GPS blackout accumulates heading error beyond ~5-7 min. Planned geofence: GNSS is an unrated input, diagnostic argument open (O27) |

---

## 2. E-stop and safety hardware

### 24V SAFETY CONTROL LOOP

```
+24V Safety Power
       │
┌──────┴──────────────────────────────────────┐
│  Physical E-Stop Buttons                    │  (Schneider XALK178, 2x NC Contacts - PLd)
└──────┬──────────────────────────────────────┘
       │
       │
┌──────┴──────────────────────────────────────┐
│  Inxpect C203A Control Unit                 │  (Safety Controller - SIL 2 / PLd)
└──────┬──────────────────────────────────────┘
       │   ▲                      ▲
       │   │                      │
       │   │ (4-Wire Direct)      │ (M12 CAN Bus)
       │   │                      │
       │ ┌─┴──────────────────┐ ┌─┴──────────────────┐
       │ │ ASO Sentir Bumpers │ │ Inxpect S101A      │
       │ │ (x2) PLd           │ │ Radars (x3) PLd    │
       │ └────────────────────┘ └────────────────────┘
       │
       │
       │
       ├──────────────────────┐
       │                      │
┌──────┴───────────┐   ┌──────┴───────────┐
│ SW180 #1 Coil    │   │ SW180 #2 Coil    │  (24V DC Actuation Coils - PLd)
│ [TVS Suppressor] │   │ [TVS Suppressor] │
└──────┬───────────┘   └──────┬───────────┘
       │                      │
───────┴──────────────────────┴──────────────────────────────── GND (24V)
```

Planned geofence entry point into this loop: OPEN (O24). Not shown.

### 48V TRACTION POWER BUS
```

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

### Core - PLd

| # | Component | Model / Supplier | Role in loop | Key spec | Status |
|---|---|---|---|---|---|
| 1 | Physical E-stops ×2 | Schneider XALK178, [Kempston Controls](https://www.kempstoncontrols.co.uk/XALK178/Schneider/sku/479749), included in RTU pricing | Input (series NC) | 2×NC contacts | Selected. IP68 upgrade required, confirm variant (O7) |
| 2 | Bumper ×2 | [ASO Sentir](https://www.automation24.co.uk/safety-bumper-aso-sentir-1701-114-60-100-l4-0?orderCode=135), 4-wire, £318 front + £318 rear = £636 (ex VAT) | Input | 4-wire fail-safe loop | Selected. Wired into Inxpect C203A. [2006/42/EG](https://asosafety.com/en/produkt/sentir-bumper-100-120-l-o-2/), EN ISO 13856-3:2013, 2011/65/EU. Price to be confirmed (O21) |
| 3 | Human-detection radar sensors ×3 | Inxpect S101A ×3 (Fortop code IT100006, Inxpect 90202011), [Fortop UK](https://shop.fortop.co.uk/en/en/inxpect-it100006-s101a-ul-radar-sensor-90202011.html), £510.00 each = £1,530.00 (ex VAT) | Input (human detection) | 24 GHz FMCW radar, SIL 2 / PL d; 0 to 4 m range, min. set distance 1 m; FOV 110°×30° (wide), max target speed 1.6 m/s; IP67; −30 to +60 °C; CAN | Manufacturer instruction manual (Inxpect SAF-IM-100S_7_00111_en_v1.6) confirms mobile/vehicle-mounted use, with a dedicated mobile-applications validation procedure (§8.4.2) and installation guidance for sensors on moving/vibrating parts. Mounting geometry not fixed (O20) |
| 4 | Radar/bumper control unit | Inxpect C203A ×1 (Fortop code IT100024, Inxpect 90304011), [Fortop UK](https://shop.fortop.co.uk/en/en/c203a-ul-control-unit-200-series-it100024-90304011.html), £570.00 (ex VAT) | Logic (radar, bumper) | Connects up to 6 sensors (3 in use); digital inputs and safety outputs; 24V | Selected. Paired with S101A |
| 5 | Output contactors ×2 | Albright SW180 24V ×2 (series, 48V B+ bus), [Arc Components](https://www.arc-components.com/sw180-3-albright-single-acting-solenoid-contactor-24v-intermittent.html), £74.69 each = £149.38 (ex VAT) + [aux micro-switch kit](https://www.arc-components.com/auxiliary-micro-switches-for-albright-contactors.html) £32 each = £64 (ex VAT) | Output | 200A cont/400A peak, magnetic blowout, silver alloy contacts, TVS suppressors | WIP (O6) |
| | *Subtotal excl. 3× radar sensors* | | | | *£1,419.38 (+£283.88 VAT)* |
| | **Total** | **£2,949.38** (+£589.88 VAT) | | | |

### Supplemental - outside the rated function

| # | Component | Model / Supplier | Role | Note | Status |
|---|---|---|---|---|---|
| 1 | IDEM GLM wire rope tether pull switch | [IDEM 143052 GLM 2NC 2NO M20](https://www.seltec.co.uk/products/details/19877.html) £77.92 excl. VAT (£93.50 incl. VAT), Seltec | Input, when connected | Die-cast, up to 30 to 50 m rope span, 2NC/2NO. Only for demonstrations operated without radar fitted or configured for the space. Not in the baseline Core series chain | Optional, demonstrations only |
| 2 | Motion alarm/beacon | [Brigade SA-BBS-97](https://www.beaconsandlightbars.co.uk/product/brigade-electronics-brigade-sa-bbs-97-77-97db-smart-bbs-tek-white-sound-reversing-alarm-pn-sa-bbs-9-17914), £95, + [rotating LED ~£40](https://www.compass24.com/led-3600-rotating-beacon-flat-396940/black), £135.00 total | Not in stop function. Avoidance measure supporting P1 (§6) | 24V, wire to motion state (O8) | Required for the PLr d basis in §6 |
| | **Total** | **£212.92** (mixed VAT basis) | | | |

### Planned - independent geofence (not in current Core)

Not in the BOM totals above. No prices recorded. Architecture in §5.

| # | Component | Source | Role | Key spec (ST product page unless noted) | Status |
|---|---|---|---|---|---|
| 1 | STEVAL-SILPLC01 ×2 | [ST](https://www.st.com/en/evaluation-tools/steval-silplc01.html) | Geofence channel A and B | STM32H723VG (up to 550 MHz); 1oo2 architecture; CLT03-2Q3 dual-channel digital input; two IPS160HF high-side outputs, 2.5 A each; supply 24 to 36 V (max 60 V); X-CUBE-STL-H7 v1.2.0 self-test library (TÜV Rheinland); RS485 PHY listed. ST states hardware assessed by TÜV Italia against SIL 2 / PL d (random failure rates, hardware systematic capability, architectural constraints) | Planned. FMEDA and assessment report available only under NDA (O26) |
| 2 | RTK GNSS receiver ×2 | u-blox F9P (per GeofenceSafely README) | GNSS input per channel | Treated as untrusted, no safety certification (README) | Planned. Separate receiver per channel to be confirmed (O27) |

---

## 3. Monitoring & Supervisory

| Item | Status | Notes |
|---|---|---|
| `sentor` (`sowbot_monitor.yaml`) | WIP, ~75% | monitors e-stop, bumpers, battery, camera, odom, neo heartbeat |
| `sentor_node.py` wired into `devkit.launch.py` | DONE | |
| `sentor` hardware smoke test | TO DO | validated in sim only (O10) |
| Battery voltage cutoff threshold | TO DO | marked `# TODO: CONFIRM` in `sowbot_monitor.yaml`, no value set (O1) |
| `ros2_medkit` black-box logging | DONE | |
| Aggregation layer + `/safety/level` | TO DO | O17 to O19 |
| ArduPilot fence/failsafe state to `/safety/level` | TO DO | Input once the §5 bridge is live, O19, O22 |

Per §0: `sentor` and the software E-stop topics are diagnostic/supervisory. They are not part of the rated safety function.

---

## 4. Perception

| Item | Status | Notes |
|---|---|---|
| Radar human detection (Inxpect S101A ×3 + C203A) | Core, in-scope per §0 | Mounting geometry not fixed (O20); build/install/validate TO DO (O14); mobile-application validation procedure defined by manufacturer manual, see §2 Core and H4 |
| Livestock false-positive tolerance | AGREED | Fine for the detector to stop on livestock, safe default |

---

## 5. Controller path

Sequence: ESP32/Lizard (now, with DroneCAN added) then ArduPilot on LEVIA-H7. Both are supervisory per §0.

| Item | Status | Notes |
|---|---|---|
| ESP32 + Lizard DSL | DONE, current. Stays until the ArduPilot cutover | hard real-time, but ESP-IDF quality. Planned: DroneCAN for motor drivers via libcanard on the ESP-IDF TWAI driver (O30) |
| ArduPilot Rover (GUIDED mode) on LEVIA-H7 ([piecol/LEVIA-H7](https://github.com/piecol/LEVIA-H7), STM32H743) | PLANNED, next | Replaces the RTU Master Controller as ArduPilot host; remaining RTU role to be confirmed (O31). Motion execution, geofence, failsafes (EKF variance, GCS/GPS loss). Board per its README: six-layer, 8 motor outputs, dual ICM-42688-P, DPS368, IST8310, one CAN interface (selectable 120 Ω termination), UART RC input, I²C and SPI expansion, USB-C, up to 6S input, CERN-OHL-S-2.0. README states the design is unvalidated, bring-up and flight testing pending. README names the MatekH743 ArduPilot hwdef as its reference; a LEVIA-specific hwdef is not confirmed (O31) |
| ArduPilot-side open items carried over (from RTU) | OPEN | `GPS1_TYPE=14` not set; EKF-origin-on-boot behaviour with `GPS_INPUT` as sole GPS source not resolved; physical MAVLink port now to be identified on LEVIA-H7 (O22 d, O31). See `devkit_mavlink_bridge` ([Agroecology-Lab/feldfreund_devkit_ros](https://github.com/Agroecology-Lab/feldfreund_devkit_ros), `caatinga-dev` branch) and `research/ardurover.md` in `Sowbot_Data` for the full TODO list |
| FusionCore (third-party, [manankharwar/fusioncore](https://github.com/manankharwar/fusioncore)), 23-state UKF fusing IMU, wheel encoders, GPS, visual SLAM | WIP, integration | Localisation/estimation layer. Not our code; Apache 2.0, published (arXiv 2605.25239). Feeds fused pose to ArduPilot as `GPS_INPUT` (GPS1_TYPE=14). Documented limits relevant to H5: yaw unobservable from IMU+encoder+GPS alone without magnetometer/dual-antenna heading; GPS blackout beyond ~5-7 min accumulates heading error |
| `devkit_mavlink_bridge` (ROS 2 to MAVLink) | WIP, outbound half only | `cmd_vel` (Twist) to `SET_POSITION_TARGET_LOCAL_NED`, republished every 0.5s (inside ArduPilot's 3.0s `GUID_TIMEOUT`). Inbound half (FusionCore pose to `GPS_INPUT`) not implemented, blocked on confirming FusionCore's output topic/type/rate |
| Cerebri on Zephyr & [FRDM-A-S32K358](https://www.nxp.com/design/design-center/development-boards-and-designs/FRDM-A-S32K358) (dual 32-bit Arm Cortex-M7 cores in lockstep, ASIL D) | NOT SELECTED. Not the controller-path target | [Agroecology-Lab/cerebri](https://github.com/Agroecology-Lab/cerebri). Its control-loop modules (`estimate.c`/EKF, `velocity.c`, `position.c`, `mixing.c`, `fsm.c`) are functionally superseded by FusionCore (estimation) and ArduPilot GUIDED mode (motion execution, mode/mission FSM). No identified remaining role pending a decision (O23). Not deleted or formally deprecated; do not resume `fsm.c`/`mixing.c`/`velocity.c`/`position.c` work before O23 is resolved. S32K358 is no longer the geofence target (see below, O32) |

**Defence-in-depth geofencing:** FusionCore fused pose, then ArduPilot Rover's native GUIDED-mode fence logic (boundary definition, breach action, EKF/GPS failsafe handling), then bridged into `/safety/level` per §3. This is **not** a certified or rated function: no MISRA, lockstep, or IEC 61508/ISO 26262 certification applies. ArduPilot's fence and failsafe logic is mature, widely deployed, open-source, but not independently certified to any standard in §6. Suitability rests on field verification (O22).

Per §0: this entire layer, whichever components it comprises, is out of scope for the PLd calculation. Its value is defence-in-depth (EKF-based fault detection, geofencing, mission execution), not certifiability of the rated stop function, which the hardwired 24V loop provides independently. It does not affect the §6 rating.

### Independent geofence (planned)

Status: PLANNED, later. Not started. Sits alongside the ArduPilot fence above, not in place of it. Source: [samuk/GeofenceSafely](https://github.com/samuk/GeofenceSafely) README.

```
RTK GNSS A ──► STEVAL-SILPLC01 A ──┐ output A
                  ▲   │ cross-check link
                  │   ▼
RTK GNSS B ──► STEVAL-SILPLC01 B ──┤ output B
                                   ▼
        entry into 24V loop / SW180 coils: OPEN (O24)
        (48V isolation: SW180 #1 + #2, §2)
```

Design requirements (GeofenceSafely README):
- Bare-metal MISRA C, cppcheck MISRA addon and clang-tidy in CI.
- GNSS in as UBX binary, not NMEA. Receiver treated as untrusted. Checks: UBX Fletcher checksum, message timeout, fix type and validity flags, accuracy estimates, jamming/spoofing indicators. Missing or stale message = outside the fence.
- Channels cross-check each other's position over a separate link.
- GNSS alone is a weak input for PLd if spoofing or multipath are in scope. An independent plausibility source (odometry or IMU) is to be considered (O27).
- Fail-safe: loss of power, clock or software must de-energise the motor. Dynamic (toggling) enable, not a static level.
- Two independent ways to cut 48V; either channel alone must trip (Category 3).
- Read back actual shutoff state; test it periodically.
- Geofence data stored as two CRC-protected copies, checked at boot and periodically.
- Debug access locked in production.

Hardware change from the README: README targets FRDM-A-S32K358 and lists S32K358 features (FCCU, STCU2 LBIST/MBIST, lockstep). These do not apply to the STM32H723 on the STEVAL-SILPLC01, where X-CUBE-STL-H7 provides the MCU self-test (O26, O32).

---

## 6. Regulatory compliance

No compliance claimed. Reference standards only until formal assessment or audit is done.

| Standard | Domain | Relevance | Legal status | Extent of compliance required |
|---|---|---|---|---|
| ISO 18497-1:2024 | Partially automated/semi-autonomous/autonomous ag machinery, design principles and vocabulary | Supersedes ISO 18497:2018. General design/verification/validation/information-for-use principles | Voluntary (harmonised standard route to EHSR conformity) | Primary standard for this product class. Required in substance before Phase 3 Declaration of Conformity; informal reference only at Phase 1 |
| ISO 18497-2:2024 | Design principles for obstacle protection systems | Governs H4 (radar/bumper human detection) | Voluntary (harmonised standard route to EHSR conformity) | Same footing as Part 1. Design-principle reference now, substantive compliance expected before Phase 3 |
| ISO 18497-3:2024 | Autonomous operating zones | Governs operating-area containment/geofencing design; tracked as H5 (§1), implemented via FusionCore + ArduPilot fence (§5); independent dual-channel geofence planned (§5) | Voluntary (harmonised standard route to EHSR conformity) | Design reference at Phase 1. Boundary excursion is a significant hazard under ISO 12100, designed per Part 3 §4.2 and verified per Part 4. Hazard treatment write-up, field verification (O22) and residual-risk disclosure in Annex VI documents (O16) required before Phase 3 |
| ISO 18497-4:2024 | Verification methods and validation principles | Phase 3 conformity assessment evidence; also the verification method Part 3 §4.2 calls for | Voluntary (harmonised standard route to EHSR conformity) | Not actioned. Becomes relevant when Phase 3 must demonstrate compliance to an assessor, and for geofence verification under O22 |
| ISO 25119 / AgPL | Tractor and ag electronics functional safety | AgPL target for motor-stop interlocks | Voluntary, written for tractors/conventional ag electronics, not robots | Design reference only. Not a certification target |
| ISO 13849-1 / PL | Machinery safety, control systems | PLd target for the E-stop, radar, bumper and contactor circuit | Voluntary (harmonised standard route to EHSR conformity) | Load-bearing for the §6 calculation regardless of phase. Full SISTEMA run (CCF ≥65, DCavg, PFHd) required before any Declaration of Conformity; not required for Declaration of Incorporation. Planned geofence is a separate safety function with its own PLr and SISTEMA run (O25) |
| ISO 3691-4 | AGV obstacle detection | Clearance rules, braking distance, detection envelope sizing | Voluntary, written for AGVs/industrial trucks, not field robots | Methodology reference only (e.g. S = KT+C sizing). Not the governing standard for this product class |
| IEC 61508 | Functional safety, E/E/PE systems | Reference for safety-controller integration (C203A, SIL 2), the STEVAL-SILPLC01 assessment basis (ST states IEC 61508, EN 62061, EN ISO 13849-1/-2) and any controller-path role decided under O23 | Voluntary | Design reference. Not independently audited at Phase 1 |
| ISO 21448 (SOTIF) | Safety of the intended functionality | Vision/radar degradation: mud, dust, glare, crop clutter (see H4) | Voluntary | Design reference only. No formal SOTIF process required at Phase 1 |
| UK SMSR 2008 / EU Machinery Directive 2006/42/EC | Machinery placing-on-market | Product classification, partly completed machinery status | Statutory | Mandatory now. Requires Annex VI assembly instructions + Declaration of Incorporation before any unit ships (O16). No CE/UKCA marking or third-party certification required at this classification |
| EU Machinery Regulation (EU) 2023/1230 | Machinery placing-on-market | Successor to 2006/42/EC; software as safety component, source code/control logic in technical documentation | Statutory, applies from 20 Jan 2027 | Mandatory for EU sales from 20 Jan 2027 (Northern Ireland: confirm applicable date). GB continues CE recognition and is aligning SMSR 2008 technically |

### Product classification

The devkit as shipped is **"partly completed machinery"** under both regimes tracked:
- **UK**: Supply of Machinery (Safety) Regulations 2008 (SI 2008/1597)
- **EU**: Machinery Directive 2006/42/EC (until 20 January 2027), then Machinery Regulation (EU) 2023/1230

This classification does not require a CE/UKCA mark or third-party certification at Phase 1, consistent with §9. It does require, before any unit ships: assembly instructions (Annex VI) and a Declaration of Incorporation (not a Declaration of Conformity) stating which essential health and safety requirements are met by the shipped components and that the unit must not be put into service until fully assembled (O16).

The EU Machinery Regulation 2023/1230 replaces the Directive from 20 January 2027. For EU sales after that date, documentation must be built against the Regulation (allows digital assembly instructions/Declaration of Incorporation, adds cybersecurity requirements, requires software update logging). No UK divergence announced; monitor.

### Functional safety calculation (draft)

**Target:** ISO 13849-1 Performance Level d (PLd), achieved via Core detection and interruption (E-stops, radar, bumper, contactors).

**Required PL (ISO 13849-1 Annex A risk graph):**
- S2: severe, irreversible injury (H1)
- F2: frequent to continuous exposure (H1)
- P1: avoidance possible. Basis: audible and visual motion warning (§2 Supplemental 2) and low vehicle speed (0.1 to 1.6 m/s range, O20)
- Result: PLr d
- Without a supported P1, P2 applies and PLr is e, which Core does not meet. P1 justification tracked in O9.

Configurations without full Core are outside this calculation. No reduced PLr is claimed for them.

**In-scope components:** §2 Core table (physical E-stops, bumper, radar ×3 + C203A, output contactors), per §0. The IDEM GLM tether enters the series chain only when connected for demonstration operation without radar, and is not part of the baseline calculation. FusionCore, ArduPilot, LEVIA-H7, ESP32/Lizard, the MAVLink bridge, any cerebri role (§5) and the planned geofence (until O25 is closed) are not in scope.

**Architecture:**
- Category 3: dual-channel inputs (E-stops 2×NC, bumper 4-wire), logic internal to the C203A, two series SW180 contactors on the 48V bus.
- Ratings: E-stops, bumper, radar and C203A are stated as PLd in §2. The SW180 output stage has no stated PL and is assessed by calculation from B10d (O3, O5).
- Diagnostic coverage (DCavg): stated range 60% to <90% (Low), pending SW180 aux-microswitch EDM verification (O6). Not a fixed number; needs pinning down before the SISTEMA run.
- Common cause failure (CCF): Annex F scoring applies (≥65 points required); addressed via channel isolation, overvoltage protection, physical wiring separation. Score not computed (O4). Scoring must account for radar and bumper sharing the C203A logic unit.

**Combination rule:** total PFHd and PL ceiling are set by the worst-performing element in the series chain (E-stops, C203A [bumper + radar inputs], dual SW180s). No element may rate below the overall target. The combined system PLd is not confirmed until the SISTEMA run (O5).

**Planned geofence:** separate safety function, not part of the H1 calculation above. Needs its own S/F/P, PLr, architecture and SISTEMA run (O25). If it drives the SW180 coils, its effect on the shared output stage is assessed there (O24).

---

## 7. Non-traction actuators (OPEN, scope flag)

This roadmap covers the traction/motor-stop function only. Any actuator outside that scope (implement, lift, arm, PTO-equivalent) has no hazard analysis or safety architecture defined here and must not be assumed covered by §2's rating. Add a dedicated section before any such actuator is fielded.

---

## 8. Open items register

Ordered roughly by build sequence.

| ID | Item | Blocks | Status |
|---|---|---|---|
| O1 | Confirm battery voltage cutoff threshold | `sowbot_monitor.yaml` completion | TO DO |
| O2 | Obtain manufacturer reliability data (PL, PFHd, B10d/MTTFd where applicable) for XALK178, ASO Sentir, Inxpect S101A and C203A | §6 SISTEMA run | TO DO |
| O3 | SW180 contactor: document B10d under expected traction switching loads; record well-tried component status (ISO 13849-2) | §6 SISTEMA run | TO DO |
| O4 | Complete formal ISO 13849-1 Annex F CCF checklist to ≥65 points, including radar/bumper sharing the C203A logic unit | §6 SISTEMA run | TO DO |
| O5 | Run final parameter set through SISTEMA (or equivalent) for finalised PFHd | Phase 2 gate | TO DO (depends on O2, O3, O4, O6, and O15 if demo tether used) |
| O6 | Confirm C203A safety output rating against SW180 coil current (24V, TVS suppressed); confirm each coil is driven from a separate C203A safety output (Cat 3 channel independence); confirm C203A input for SW180 aux-microswitch feedback (EDM) | §2 Core component 5, H3, §6 DCavg | TO DO |
| O7 | Build and test the hard-wired 24V loop (E-stops, bumpers, C203A, SW180) on current hardware; confirm E-stop IP68 variant | §2 | WIP |
| O8 | Wire motion alarm and flashing LED to motion state | §2 Supplemental 2, O9 | TO DO |
| O9 | Document S/F/P parameter justification for H1, including the P1 basis (motion warning, vehicle speed range). If P1 is not supported, PLr is e and Core does not meet it | §6 required PL | TO DO |
| O10 | `sentor` hardware smoke test (sim-only so far) | §3 | TO DO |
| O11 | Decide resume-confirmation behaviour after E-stop trigger | §2 | WIP |
| O12 | Define fail-state for steering on E-stop trigger (undocumented) | §2 fail-state definition, H2 | OPEN |
| O13 | Assess rollaway risk on slope (H2): confirm 40:1 worm gear ratio and self-locking against gearbox datasheet; confirm operating slope range; decide if a positive parking brake is needed as backup | §1 H2 | OPEN |
| O14 | Install and validate Inxpect S101A ×3 + C203A on-vehicle using the manufacturer's mobile-application procedure (manual §8.4.2); confirm S101A detection response time for H1 | §4, §1 H1, H4 | TO DO |
| O15 | IDEM GLM tether: confirm rope length needed; obtain B10d/MTTFd/PFHd | Demo-mode operation only | TO DO, low priority |
| O16 | Prepare Declaration of Incorporation and assembly instructions (Annex VI) for partly-completed-machinery shipment, per §6 Product classification. Includes H5 boundary-excursion residual-risk disclosure (O22) | Any unit shipment | TO DO |
| O17 | Adopt `diagnostic_aggregator` between `sentor`'s per-topic output and a single ordered `/safety/level`, replacing the flat `/safety/heartbeat` + `/warning/heartbeat` pair | §3 | TO DO |
| O18 | Define per-monitor de-escalation policy (auto-clear vs. manual re-arm), extending O11 to every monitor feeding `/safety/level` | §3, O11 | TO DO |
| O19 | Wire mission executor and Nav2 lifecycle to threshold on `/safety/level` rather than each subscribing to raw E-stop/bumper/node-liveliness topics. Integration point for ArduPilot fence/failsafe state (O22) | §3 | TO DO |
| O20 | Define 3-sensor radar mounting geometry (positions, boresight angles). Re-derive FOV gap and detection-envelope sizing using the H1 detection-to-stop latency and the ISO 3691-4 method. Incorporate mobile-application validation requirements: sensor field of view must reach test positions (dangerous-area boundaries, inter-sensor gaps, partially hidden positions) at 0.1 to 1.6 m/s; check the manufacturer's Low anti-masking sensitivity setting (sensors on moving parts) against the chosen mounting | §1 H1, H4, §2 Core, O9 | OPEN |
| O21 | Confirm ASO Sentir bumper unit price against supplier quote (BOM shows £318 each) | §2 Core BOM | OPEN |
| O22 | Geofence (H5) verification per ISO 18497-3/-4: (a) confirm ArduPilot fence breach action is an actual stop/hold, not report-only; (b) field-test boundary approach across the speed range and worst-case GNSS-degradation scenarios (multipath, brief blackout, EKF-origin-on-boot with `GPS_INPUT` as sole GPS source); (c) exercise ArduPilot's GPS-glitch/EKF-failsafe handling, not just nominal-fix behaviour; (d) confirm `GPS1_TYPE=14` and the physical MAVLink port on the ArduPilot host (RTU, moving to LEVIA-H7, see O31) (blocks all of the above, see §5); (e) wire fence/failsafe state into `/safety/level` per O19; (f) draft the residual-risk operator disclosure for O16 | §1 H5, §6 ISO 18497-3/-4, O16, O19 | OPEN |
| O23 | Decide cerebri's role (retain with a defined function, or formally deprecate). If Zephyr/S32K358 is retained, track Zephyr IEC 61508 SEooC status | §5 | OPEN |
| O24 | Geofence output integration: choose where each channel enters the 24V loop (e.g. in series with each SW180 coil supply, or as inputs to the C203A; the latter shares the C203A logic unit). Confirm IPS160HF output (2.5 A per ST) against SW180 coil current with TVS suppression (extends O6). Define fail-safe on power/clock/software loss, dynamic enable, and shutoff readback (EDM) | §2 Planned, §5 | OPEN |
| O25 | Define the geofence as a separate safety function: S/F/P, PLr, architecture (per-board 1oo2 vs two-board Category 3), own SISTEMA run. Decide whether it joins Core or stays supplemental. Update §0 and §6 accordingly | §0, §6, H5 | OPEN |
| O26 | STEVAL-SILPLC01: obtain TÜV Italia assessment report, FMEDA and TN1395 under NDA. Confirm what the assessment covers for custom application firmware with a GNSS input (ST page does not say). Integrate X-CUBE-STL-H7 | §2 Planned, §5, O25 | TO DO |
| O27 | Geofence GNSS input: confirm separate RTK receiver per channel. Implement UBX validity, timeout, accuracy and jamming/spoofing checks and the cross-channel position compare. Decide on an independent plausibility source (odometry or IMU). Write the failure-mode argument (fix loss, RTK float, jamming, spoofing, multipath) | H5, O25 | OPEN |
| O28 | Geofence GNSS link: README specifies UART to the receiver; decision is RS485. ST lists an RS485 PHY on the STEVAL-SILPLC01 alongside EtherCAT; confirm it is usable for GNSS input or add a transceiver | §2 Planned | OPEN |
| O29 | Size the geofence boundary margin: GNSS latency + fix-loss timeout + geofence compute + shutoff and contactor drop-out + stop distance at max speed (1.6 m/s). Extends O20 | H5, O25 | OPEN |
| O30 | DroneCAN on ESP32/Lizard: add libcanard on the ESP-IDF TWAI driver; select DroneCAN motor driver; validate kinematics on the vehicle | §5 | TO DO |
| O31 | LEVIA-H7 as ArduPilot host: review the design independently (README: unvalidated, bring-up pending); bring-up and testing; confirm a LEVIA-specific ArduPilot hwdef (README references MatekH743); identify the MAVLink UART and confirm the CAN interface for DroneCAN; carry over `GPS1_TYPE=14` and EKF-origin items (O22 d); confirm the RTU's remaining role | §5, O22 | OPEN |
| O32 | Update GeofenceSafely to the STEVAL-SILPLC01 target: README targets FRDM-A-S32K358 and S32K358 safety features; its structure section lists an STM32L4 mcal and HAL (template text). STM32H723 port needed. Repository source beyond the README not reviewed for this revision | §5 | TO DO |

---

## 9. Phased rollout

### Phase 1: dev platform (current focus)
Audience: university labs, ag-tech researchers, software startups.

| Item | Status |
|---|---|
| PLd calculation | Draft in progress, §6 |
| Third-party audit | NOT DONE |
| Compliance claim in docs or marketing | NONE, correctly |
| Standards used as design reference | YES, informal |
| Liability position | User's own risk, stated in README |
| Controller path | ESP32/Lizard now (DroneCAN to be added, O30); ArduPilot on LEVIA-H7 next (O31) |
| Independent geofence (2× STEVAL-SILPLC01) | PLANNED, later. Not started (O24 to O29, O32) |

No certification work needed at this phase. Reference the standards, do not claim them.

### Phase 2: OEM modular subsystems
Audience: Academics/startups integrating Sowbot's drive/safety core.

| Item | Status |
|---|---|
| PLd calculation for E-stop, radar, bumper and contactor circuit | Draft in progress |
| SOTIF assessment for radar degradation cases | TO DO |
| IEC 61508 architecture review | TO DO |
| Third-party certification | NOT REQUIRED YET; formal internal assessment is |
| Geofence safety function definition and PLr (O25) | TO DO |

### Phase 3: commercial sale to farmers
Audience: commercial growers, farm management enterprises.

| Item | Status |
|---|---|
| Full ISO 18497 compliance | REQUIRED before sale |
| ISO 13849 PLd certification, or equivalent | REQUIRED before sale |
| Third-party safety audit | REQUIRED before sale |
| Field trial history | REQUIRED before sale |
| Insurance and liability structure | NOT YET SCOPED |
