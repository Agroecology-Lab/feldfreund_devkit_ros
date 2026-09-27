# Sowbot Safety Roadmap, v0.6.5
## Contents

- [0. Safety scoping](#0-safety-scoping)
- [1. Hazard identification](#1-hazard-identification)
- [2. E-stop and safety hardware](#2-e-stop-and-safety-hardware)
  - [24V SAFETY CONTROL LOOP](#24v-safety-control-loop)
  - [48V TRACTION POWER BUS](#48v-traction-power-bus)
  - [Core - PLd](#core---pld)
  - [Supplemental - optional, not part of baseline PLd calc](#supplemental---optional-not-part-of-baseline-pld-calc)
- [3. Monitoring & Supervisory](#3-monitoring--supervisory)
- [4. Perception](#4-perception)
- [5. Controller path](#5-controller-path)
- [6. Regulatory compliance](#6-regulatory-compliance)
  - [Product classification](#product-classification)
  - [Functional safety calculation (draft)](#functional-safety-calculation-draft)
- [7. Non-traction actuators (OPEN, scope flag)](#7-non-traction-actuators-open-scope-flag)
- [8. Open items register](#8-open-items-register)
- [9. Phased rollout](#9-phased-rollout)
  - [Phase 1: dev platform (current focus)](#phase-1-dev-platform-current-focus)
  - [Phase 2: OEM modular subsystems](#phase-2-oem-modular-subsystems)
  - [Phase 3: commercial sale to farmers](#phase-3-commercial-sale-to-farmers)
- [Revision history](#revision-history)
  
**Document control**
- Previous version: v0.6.4
- Status: living document, Phase 1 (dev platform)
- Scope: whole-vehicle emergency motor-stop function, ISO 13849-1. Does not cover implement, PTO, or manipulator safety, see §7.
- Owner: TBD (assign)

---

## 0. Safety scoping

The formal safety function is the hardwired 24V E-stop loop (§2, Core components), the human-detection radar (Inxpect S101A ×4 + C203A), and the ASO Sentir bumpers, together with the 48V traction interlock they drive. 

Everything else in this document (ROS 2 nodes, `sentor`, ESP32/STM32H7 firmware, non-radar perception, reversing alarm/beacon) is supervisory or defence-in-depth, **not** part of the rated safety function, and does not enter the PLc/PLd calculation in §6. .

This split holds regardless of microcontroller or firmware changes (ESP32 > S32K358, Lizard > CoginiPilot, ESP-IDF > Zephyr). Moving the controller firmware does not move the safety boundary. 

---

## 1. Hazard identification

Supports the S2/F2/P1 risk graph parameters in §6. Not exhaustive, add rows as identified.

| ID | Hazard | Cause | Exposure | Current mitigation | Residual risk |
|---|---|---|---|---|---|
| H1 | Crush/impact from moving vehicle | Software fault, sensor failure, operator error | Continuous during field operation (F2) | Physical bumper + E-stop loop + radar (Core, per §0) | E-stop loop response <30ms; radar detection response <100ms (catalogue figure, not S101A-specific, see O23c); combined detection-to-stop latency ~100-130ms, not yet sized against ISO 3691-4 |
| H2 | Rollaway after stop | Stop on slope, no parking brake | Not yet assessed (CONFIRM operating slope range) | Worm gear drive (assumed 40:1) self-locks tracks when unpowered | OPEN, self-locking ratio assumed, not yet confirmed against gearbox datasheet; no positive parking brake in BOM as backup |
| H3 | Undetected E-stop hardware degradation | Contactor welding, relay failure over time | Continuous | PRSU/2 EDM loop via SW180 aux microswitches | Covered by DCavg in §6, pending final CCF/SISTEMA figures |
| H4 | False negative on human detection | Radar coverage gaps; sensor failure | Continuous once deployed near people | Inxpect S101A ×3 + C203A control unit, now Core, in-scope per §0 (resolves O24) | OPEN, mounting geometry for 4-sensor layout not yet defined (see O28); moving-vehicle suitability confirmed by manufacturer instruction manual (O23a, closed v0.6.5), mobile-application validation procedure still to be run on-vehicle; min. set distance (1m) still close to worst-case detection-envelope sizing from H1 |

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
       │ │ (x2) PLd           │ │ Radars (x2) PLd    │
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
| 1 | Physical E-stops | Schneider XALK178 ×2, [Kempston Controls](https://www.kempstoncontrols.co.uk/XALK178/Schneider/sku/479749), £29.05 each excl. VAT | Input (series NC) | 2×NC contacts | Upgrade to IP68 |
| 2 | Bumper | [ASO Sentir](https://www.automation24.co.uk/safety-bumper-aso-sentir-1701-114-60-100-l4-0?orderCode=135), 4-wire, price TBD (see O29, garbled in prior Supplemental pricing, possibly the $315 quote-only figure) | Input | 4-wire fail-safe loop | Selected. Wired into Inxpect C203A. [2006/42/EG](https://asosafety.com/en/produkt/sentir-bumper-100-120-l-o-2/), EN ISO 13856-3:2013, 2011/65/EU |
| 3 | Human-detection radar sensors ×3 | Inxpect S101A ×4 (Fortop code IT100006, Inxpect 90202011), [Fortop UK](https://shop.fortop.co.uk/en/en/inxpect-it100006-s101a-ul-radar-sensor-90202011.html), £510.00 each = £1,530.00 (VAT status not stated), delivery approx. 4 weeks | Input (human detection) | 24 GHz FMCW radar, SIL 2 / PL d; 0 to 4 m range, min. set distance 1 m; FOV 110°×30° (wide) max target speed 1.6 m/s; IP67; −30 to +60 °C; CAN | Manufacturer instruction manual (Inxpect SAF-IM-100S_7_00111_en_v1.6) confirms mobile/vehicle-mounted use, with a dedicated "mobile applications" validation procedure (§8.4.2) and installation guidance for sensors mounted on moving/vibrating parts (O23a, closed v0.6.5). Mounting geometry for the 4-sensor layout not yet fixed, see O28. |
| 4 | Radar control unit | Inxpect C203A ×1 (Fortop code IT100024, Inxpect 90304011), [Fortop UK](https://shop.fortop.co.uk/en/en/c203a-ul-control-unit-200-series-it100024-90304011.html), £570.00 (VAT status not stated) | Logic (radar) | Connects up to 6 sensors (4 in use); digital inputs and safety outputs | Selected. Pairing with S101A. Supply voltage vs. 24V safety power still unconfirmed, see O23e |
| 5 | Output contactors | Albright SW180 24V ×2 (series, 48V B+ bus), [Arc Components](https://www.arc-components.com/sw180-3-albright-single-acting-solenoid-contactor-24v-intermittent.html), £74.69 each = £149.38 excl. VAT + [aux micro-switch kit](https://www.arc-components.com/auxiliary-micro-switches-for-albright-contactors.html) £32.09 = £181.47 | Output | 200A cont/400A peak, magnetic blowout, silver alloy contacts, TVS suppressors | WIP |
| **Total** | | **£2,339.57** known (excl VAT) | | | |

### Supplemental - optional, not part of baseline PLd calc

| # | Component | Model / Supplier | Role | Note | Status |
|---|---|---|---|---|---|
| 1 | IDEM GLM wire rope tether pull switch | [IDEM 143052 GLM 2NC 2NO M20](https://www.seltec.co.uk/products/details/19877.html) £77.92 excl. VAT (£93.50 incl. VAT), Seltec | Input, when connected | Die-cast, up to 30 to 50m rope span, 2NC/2NO. **Only needed if operating demos without radar fitted/configured for the space**, not in the baseline Core series chain | For demonstrations |
| 2 | Motion alarm/beacon | [Brigade SA-BBS-97](https://www.beaconsandlightbars.co.uk/product/brigade-electronics-brigade-sa-bbs-97-77-97db-smart-bbs-tek-white-sound-reversing-alarm-pn-sa-bbs-9-17914), £95, + [rotating LED ~£40](https://www.compass24.com/led-3600-rotating-beacon-flat-396940/black), £135.00 total | Not in stop function, avoidance measure only | 24V, wire to motion state. **Can be used to justify the P1 risk-graph parameter and reduce the target to PLc for configurations not running full Core (radar+bumper)**, not needed when Core is fitted | Can be removed if full Core fitted |
| **Total** | | **£212.92** (excl. VAT, mixed) | | | |

---

## 3. Monitoring & Supervisory

| Item | Status | Notes |
|---|---|---|
| `sentor` (`sowbot_monitor.yaml`) | WIP, ~75% | monitors e-stop, bumpers, battery, camera, odom, neo heartbeat |
| `sentor_node.py` wired into `devkit.launch.py` | DONE | |
| `sentor` hardware smoke test | TO DO | validated in sim only so far |
| Battery voltage cutoff threshold | TO DO | marked `# TODO: CONFIRM` in `sowbot_monitor.yaml`, no value set |
| `ros2_medkit` black-box logging | Done | added 25-9-26 |
| Aggregation layer + `/safety/level` | TO DO | see architecture note below |

Per §0: `sentor` and the software E-stop topics are diagnostic/supervisory. They are not part of the rated safety function.

---

## 4. Perception

| Item | Status | Notes |
|---|---|---|
| Radar human detection (Inxpect S101A ×4 + C203A) | Core, in-scope per §0 (v0.6.4) | mounting geometry not yet fixed (O28); build/install/validate still TO DO; mobile-application validation procedure defined by manufacturer manual, see §2 Core and H5 |
| Livestock false-positive tolerance | AGREED | fine for the detector to stop on livestock, safe default (agreed under the thermal plan, carried over to radar) |

---

## 5. Controller path

| Item | Status | Notes |
|---|---|---|
| ESP32 + Lizard DSL | DONE, current | hard real-time, but ESP-IDF quality |
| Cerebri on Zephyr & [FRDM-A-S32K358](https://www.nxp.com/design/design-center/development-boards-and-designs/FRDM-A-S32K358) dual 32-bit Arm® Cortex®-M7 cores operating in lockstep to support ASIL D functional safety | WIP | [Agroecology-Lab/cerebri](https://github.com/Agroecology-Lab/cerebri) |

Per §0: this layer is out of scope for the PLd calculation both before and after migration. The migration's value is defence-in-depth (EKF-based failsafes, geofencing) and long-run firmware certifiability. It does not change, and does not need to change, the §6 rating.

Zephyr's own safety programme (IEC 61508 SIL 3 SEooC, concept approval granted via route 3s, ISO 26262 alignment in progress) -

## 6. Regulatory compliance

No compliance claimed. Reference standards only until formal assessment or audit is done.

| Standard | Domain | Relevance | Legal status | Extent of compliance required |
|---|---|---|---|---|
| ISO 18497-1:2024 | Partially automated/semi-autonomous/autonomous ag machinery, design principles and vocabulary | Supersedes ISO 18497:2018. General HAAM design/verification/validation/info-for-use principles | Voluntary (harmonised standard route to EHSR conformity) | Primary standard for this product class. Required in substance before Phase 3 Declaration of Conformity; informal reference only at Phase 1 |
| ISO 18497-2:2024 | Design principles for obstacle protection systems | Governs H5 (radar/bumper human detection), O23, O24 (resolved v0.6.4) | Voluntary (harmonised standard route to EHSR conformity) | Same footing as Part 1. Design-principle reference now, substantive compliance expected before Phase 3 |
| ISO 18497-3:2024 | Autonomous operating zones | Governs operating-area containment, geofencing design (currently untracked as a hazard, see §5 EKF/geofencing note) | Voluntary (harmonised standard route to EHSR conformity) | Design reference only at present. Worth reading before committing engineering time to geofencing, since it likely specifies what "defining the operating zone" needs to cover |
| ISO 18497-4:2024 | Verification methods and validation principles | Phase 3 conformity assessment evidence | Voluntary (harmonised standard route to EHSR conformity) | Not actioned yet. Becomes the relevant part once Phase 3 needs to demonstrate compliance to an assessor rather than just state it || ISO 25119 / AgPL | Tractor and ag electronics functional safety | AgPL target for motor-stop interlocks | Voluntary, written for tractors/conventional ag electronics, not robots | Design reference only. Not a certification target for this product |
| ISO 13849 / PL | Machinery safety, control systems | PLd target for E-stop, radar, and bumper circuit (raised from PLc, v0.6.4) | Voluntary (harmonised standard route to EHSR conformity) | Load-bearing for the §6 calculation regardless of phase. Full SISTEMA run (CCF ≥65, DCavg, PFHd) required before any Declaration of Conformity; not required for Declaration of Incorporation |
| ISO 3691-4 | AGV obstacle detection | Clearance rules, braking distance, detection envelope sizing | Voluntary, written for AGVs/industrial trucks, not field robots | Methodology reference only (e.g. S = KT+C sizing). Not the governing standard for this product class |
| IEC 61508 | Functional safety, E/E/PE systems | Reference for controller firmware architecture (§5) | Voluntary | Design reference for Zephyr/cerebri SEooC route (O18). Not independently audited at Phase 1 |
| ISO 21448 (SOTIF) | Safety of the intended functionality | Vision/radar degradation: mud, dust, glare, crop clutter (see H5) | Voluntary | Design reference only. No formal SOTIF process required at Phase 1 |
| UK SMSR 2008 / EU Machinery Directive 2006/42/EC | Machinery placing-on-market | Product classification, partly completed machinery status | Statutory | Mandatory now. Requires Annex VI assembly instructions + Declaration of Incorporation before any unit ships (O22, not yet done). No CE/UKCA marking or third-party certification required at this classification |
| EU Machinery Regulation (EU) 2023/1230 | Machinery placing-on-market | Successor to 2006/42/EC; software as safety component, source code/control logic in technical documentation | Statutory, in force 20 Jan 2027 | Mandatory for any EU/NI sale from 20 Jan 2027. NI adopts directly from Oct 2026. GB continues CE recognition and is aligning SMSR 2008 technically |

### Product classification

The devkit as currently shipped is **"partly completed machinery"** under both regimes tracked:
- **UK**: Supply of Machinery (Safety) Regulations 2008 (SI 2008/1597)
- **EU**: Machinery Directive 2006/42/EC (until 20 January 2027), then Machinery Regulation (EU) 2023/1230

This classification does **not** require a CE/UKCA mark or third-party certification at Phase 1, consistent with §9. It **does** require, before any unit ships: assembly instructions (Annex VI) and a Declaration of Incorporation (not a Declaration of Conformity) stating which essential health and safety requirements are met by the shipped components and that the unit must not be put into service until fully assembled. Track the Declaration of Incorporation as a deliverable, not yet in §8, add as O22.

The EU Machinery Regulation 2023/1230 replaces the Directive from 20 January 2027. If Phase 2/3 activity extends past that date for EU sales, documentation must be built against the Regulation (allows digital assembly instructions/Declaration of Incorporation, adds cybersecurity requirements relevant to H3, requires software update logging). No UK divergence announced yet; monitor.

### Functional safety calculation (draft)

**Target:** ISO 13849-1 Performance Level d (PLd), achieved directly via Core detection/interruption (E-stops, radar, bumper), not via the P1 avoidance route.
**Risk graph parameters:** S2 (severe/irreversible injury, see H1), F2 (frequent/continuous exposure), P1 (avoidance possible via white-sound alarm and flashing beacon, Supplemental only) or P2 (no avoidance signal, Core-only configuration). Core, as now built entirely from PLd-rated components, targets PLd regardless of P1/P2, making the avoidance argument unnecessary for Core. **The Supplemental light/buzzer package remains available specifically to justify a reduced PLc target (via P1) for configurations that don't run full Core**, e.g. early Phase 1 units without radar fitted.

**In-scope components:** §2 Core table (physical E-stops, bumper, radar ×3 + C203A, output contactors), per §0. The IDEM GLM tether enters the series chain only when connected for demo-mode operation without radar; not part of the baseline calculation.

**Architecture:**
- Category 3, dual-channel redundant structure across inputs, logic, and output power interlocks.
- Diagnostic coverage (DCavg): stated range 60 to 90% (Low), pending SW180 aux-microswitch EDM-loop verification. Not yet a fixed number, needs pinning down before the SISTEMA run, see §8.
- Common cause failure (CCF): Annex F scoring applies (≥65 points required); addressed via channel isolation, overvoltage protection, physical wiring separation. Score not yet computed, see §8. **CCF scoring must now also account for radar and bumper sharing the C203A logic unit, not previously assessed.**

**Combination rule:** total PFHd and PL ceiling are set by the worst-performing element in the series chain (E-Stops → C203A [bumper + radar inputs] → dual SW180s). No component may rate below the overall target. Core components are independently PLd-rated, though the combined system PLd still depends on the SISTEMA run (O6) and is not yet confirmed.


---

## 7. Non-traction actuators (OPEN, scope flag)

This roadmap covers the traction/motor-stop function only. Any future actuator outside that scope, an implement, a lift, an arm, PTO-equivalent, has no hazard analysis or safety architecture defined here and must not be assumed covered by §2's rating. Add a dedicated section before any such actuator is fielded.

---

## 8. Open items register

Single tracked list. Ordered roughly by build sequence, re-order as priorities shift.

| ID | Item | Blocks | Status |
|---|---|---|---|
| O1 | Confirm battery voltage cutoff threshold | `sowbot_monitor.yaml` completion | TO DO |
| O2 | Tapeswitch VBL + PRSU/2: obtain B10d, MTTFd, PFHd for the 4-wire fail-safe configuration | §6 SISTEMA run | TO DO. **Neither component has a public datasheet — direct vendor contact required for both. Do not conflate PRSU/2 with Tapeswitch's public PSSR-2 product (different Cat 3/PL-d/SIL 2 unit), see §2 Supplemental.** |
| O3 | Tyro Gemini 1S / Indus 1S: confirm PFHd against manufacturer's Declaration of Conformity | N/A | **CLOSED, not applicable.** Wireless pendant not implemented (v0.6.4) |
| O4 | SW180 contactor: document B10d under expected traction switching loads; record well-tried component status (ISO 13849-2) | §6 SISTEMA run | TO DO |
| O5 | Complete formal ISO 13849-1 Annex F CCF checklist to ≥65 points | §6 SISTEMA run | TO DO. Now includes radar/bumper sharing the C203A logic unit |
| O6 | Run final parameter set through SISTEMA (or equivalent) for finalised PFHd | Phase 2 gate | TO DO (depends on O2, O4, O5, O19 if demo tether used) |
| O7 | Resolve SW180 coil voltage against PRSU/2 output rating; decide single vs. redundant contactor | §2 Core, component 5 | WIP |
| O8 | Build and test physical hard-wired E-stop and bumper E-stops on current hardware | §2 | WIP |
| O9 | Check wireless pendant PL-c claim against declaration of conformity in practice (bench test) | N/A | **CLOSED, not applicable.** Wireless pendant not implemented (v0.6.4) |
| O10 | Add reversing alarm and flashing LED, wired to motion state | §2 Supplemental, component 2 | TO DO |
| O11 | `sentor` hardware smoke test (currently sim-only) | §3 | TO DO |
| O12 | Add `ros2_medkit` black-box logging | §3 | TO DO |
| O13 | Decide resume-confirmation behaviour after E-stop trigger | §2 | WIP |
| O14 | Define fail-state for steering on E-stop trigger (currently undocumented) | §2 fail-state definition, H2 | OPEN |
| O15 | Assess rollaway risk on slope (H2): confirm 40:1 worm gear ratio and self-locking against gearbox datasheet; confirm operating slope range; decide if a positive parking brake is still needed as backup | §1 H2 | OPEN |
| O16 | Assess RF jamming/spoofing risk for wireless pendant (H3) | N/A | **CLOSED, not applicable.** No RF input remains in the safety function (v0.6.4) |
| O17 | Human-detection hardware: Inxpect S101A ×4 (up from ×2) + C203A, now Core; install and validate (H5) | §4, §1 H5 | TO DO |
| O18 | Track Zephyr IEC 61508 SEooC certification status directly rather than as an open-ended reference | §5 | Ongoing |
| O19 | IDEM GLM wire rope tether: confirm rope length needed; obtain B10d/MTTFd/PFHd | Demo-mode operation only | TO DO, lower priority now Supplemental |
| O20 | Obtain written confirmation from Cattron of the Gemini 1S's fail-state on total signal loss (fails to commanded stop vs. holds last state) | N/A | **CLOSED, not applicable.** Wireless pendant not implemented (v0.6.4) |
| O21 | Clarify with Cattron which reaction-time figure governs system response: Indus 1S's 3-tier (0.5/1.0/1.5s) vs Gemini 1S's 4-tier (0.5/1.0/1.5/2.0s) spec | N/A | **CLOSED, not applicable.** Wireless pendant not implemented (v0.6.4) |
| O22 | Prepare Declaration of Incorporation and assembly instructions (Annex VI) for partly-completed-machinery shipment, per §6 Product classification | Any unit shipment | TO DO |
| O23 | Confirm with Inxpect / Fortop: (a) S101A is approved for mounting on a moving vehicle and detects a stationary person ahead of a moving platform; (b) S101A pairs with C203A; (c) sensor-to-output response time against H1; (d) outdoor use; (e) C203A supply voltage against the 24V safety power | §1 H5, §2 Core 3 and 4 | (a) **CLOSED, v0.6.5.** Confirmed via manufacturer instruction manual (Inxpect SRE 100 Series / SAF-IM-100S_7_00111_en_v1.6): mobile/vehicle-mounted use is a documented application with its own validation procedure (§8.4.2, "mobile applications" test: stationary target, machinery/vehicle speed 0.1–1.6 m/s) and installation guidance for sensors on moving/vibrating parts (§3.10.7, §8.1.1, §8.2.3). On-vehicle validation per that procedure is still TO DO, tracked under O17. (b) **CLOSED**, confirmed via Inxpect catalogue (100 SERIES = C203A + up to 6× S101A). (c) PROVISIONALLY ANSWERED, catalogue states <100ms for the radar line generally, not S101A-specific, carried into H1. (d) REASONABLY SUPPORTED by IP67/−30 to +60°C spec and general environmental-resistance claims, not formally confirmed. (e) OPEN, no datasheet found |
| O24 | Decide whether the radar output stays supervisory (ROS layer only, per §0) or is wired into the PRSU/2 stop loop. If the latter, re-open §0 and re-run §6 | §0, §6 | **CLOSED, resolved v0.6.4.** Radar wired into stop loop; §0 and §6 updated |
| O25 | Adopt `diagnostic_aggregator` between `sentor`'s per-topic output and a single ordered `/safety/level`, replacing the flat `/safety/heartbeat` + `/warning/heartbeat` pair | §3 | TO DO |
| O26 | Define per-monitor de-escalation policy (auto-clear vs. requires manual re-arm), extending O13's E-stop-specific rule to every monitor feeding `/safety/level` | §3, O13 | TO DO |
| O27 | Wire mission executor and Nav2 lifecycle to threshold on `/safety/level` rather than each subscribing to raw E-stop/bumper/node-liveliness topics individually | §3 | TO DO |
| O28 | Define 4-sensor radar mounting geometry (positions, boresight angles) and re-derive FOV gap/detection-envelope sizing; supersedes the earlier 3-sensor/single-pole/11°-gap estimate. **Must incorporate the mobile-application validation requirements confirmed under O23a**: sensor field of view must reach test positions (dangerous-area boundaries, inter-sensor gaps, partially-hidden positions) while the vehicle moves at 0.1–1.6 m/s, and the manufacturer's Low anti-masking sensitivity setting (for sensors on moving parts) needs checking against the chosen mounting once geometry is fixed. | §1 H5, §2 Core | OPEN |
| O29 | Confirm ASO Sentir bumper unit price; source pricing in v0.7 and earlier was garbled/ambiguous in the Core/Supplemental split, possibly the $315 quote-only figure | §2 Core BOM | OPEN |

---

## 9. Phased rollout

### Phase 1: dev platform (current focus)
Audience: university labs, ag-tech researchers, software startups.

| Item | Status |
|---|---|
| PLd calculation | Draft in progress, §6 (raised from PLc, v0.6.4) |
| Third-party audit | NOT DONE |
| Compliance claim in docs or marketing | NONE, correctly |
| Standards used as design reference | YES, informal |
| Liability position | User's own risk, stated in README |

No certification work needed at this phase. Reference the standards, do not claim them.

### Phase 2: OEM modular subsystems
Audience: Academics/startups integrating Sowbot's drive/safety core.

| Item | Status |
|---|---|
| PLd calculation for E-stop, radar, and bumper circuit | Draft in progress |
| SOTIF assessment for vision/radar degradation cases | TO DO |
| IEC 61508 architecture review | TO DO |
| Third-party certification | NOT REQUIRED YET; formal internal assessment is |

### Phase 3: commercial sale to farmers
Audience: commercial growers, farm management enterprises.

| Item | Status |
|---|---|
| Full ISO 18497 compliance | REQUIRED before sale |
| ISO 13849 PLd certification, or equivalent | REQUIRED before sale |
| Third-party safety audit | REQUIRED before sale |
| Field trial history | REQUIRED before sale |
| Insurance and liability structure | NOT YET SCOPED |

---

## Revision history

- **v0.6.5**: closed O23(a) — mobile/vehicle-mounted use of the Inxpect S101A confirmed directly from the manufacturer's instruction manual (Inxpect SRE 100 Series, SAF-IM-100S_7_00111_en_v1.6), which defines a dedicated mobile-applications validation procedure and installation guidance for sensors on moving/vibrating machinery parts. Updated H4, §2 Core component 3, §4, and O28 accordingly; O28 now explicitly incorporates the manual's mobile-application validation requirements (target speed range, test positions, anti-masking setting for moving mounts). On-vehicle validation itself remains open, tracked under O17.
- **v0.6.4**: radar (Inxpect S101A, ×2→×4) and ASO Sentir bumpers moved from Supplemental to Core, resolving O24, radar is now in the rated stop loop, §0/§6 updated, target raised PLc→PLd. Wireless pendant (Tyro Indus 1S/Gemini 1S) removed from the design, NOT IMPLEMENTED, closes H3 and O3/O9/O16/O20/O21. IDEM GLM tether moved to Supplemental, relevant only for demo operation without radar. Reversing alarm/beacon moved to Supplemental, retained as an optional route to a reduced PLc target via the P1 risk-graph parameter for non-Core configurations. Added O28 (4-sensor mounting geometry, supersedes earlier 3-sensor/11°-gap estimate) and O29 (bumper pricing gap). Answered O23(b) (confirmed) and O23(c)/(d) (provisional/supported) from Inxpect catalogue material; O23(a) and O23(e) remain open.
- **v0.7**: added §3 supervisory architecture note — adopting `diagnostic_aggregator` for level aggregation over the `RobotStateMachine`/`sentor_guard` path (found unstable in sentor's own upstream history, Dec 2025); added O25–O27 for the aggregation layer, per-level de-escalation policy, and wiring the mission executor/Nav2 to a new `/safety/level` topic.
- **v0.6.2**: added Inxpect S101A ×2 (front and rear) and one C203A control unit, both from Fortop UK, to §2 Supplemental as components 4 and 5 (£510.00 each and £570.00) and updated the Supplemental total; replaced the thermal detection plan (MLX90640/MLX90614) with the radar in H5, §4, the §6 SOTIF row, §9 Phase 2 and O17; excluded the radar from the §6 in-scope list pending O24; added O23 (application and compatibility checks with Inxpect/Fortop) and O24 (whether the radar output stays supervisory or enters the stop loop, which would re-open §0 and §6).
- **v0.6.1**: added wireless pendant datasheet review notes to §2 (reaction-time tier mismatch between Indus 1S and Gemini 1S, internal battery-life inconsistency in the Indus 1S sheet, IP66-vs-IP65 discrepancy between website and datasheet); flagged that Tapeswitch's public PSSR-2 product must not be conflated with PRSU/2 (§2 Supplemental, O2); updated H3 and the §6 cybersecurity note with the fail-state-on-signal-loss gap; added new open items O20 (Gemini 1S fail-state confirmation, blocking), O21 (reaction-time reconciliation), O22 (Declaration of Incorporation / assembly instructions); added §6 Product classification subsection covering UK Supply of Machinery (Safety) Regulations 2008, EU Machinery Directive 2006/42/EC, and the 20 January 2027 transition to EU Machinery Regulation (EU) 2023/1230.
- **v0.6**: updated H2 and O15 with the drivetrain finding that the worm gear self-locks the tracks when unpowered, assumed ratio 40:1 pending confirmation against the actual gearbox datasheet. H2 stays OPEN until that's confirmed and a decision is made on whether a positive parking brake is still needed as backup.
- **v0.5**: restored the v0.3 Core/Supplemental table split in §2, including the IDEM GLM wire rope tether switch (Core, component 4) that v0.4 dropped when it merged the two tables into one; added the tether switch back as O19 in the open items register. Kept v0.4's §0 safety scoping statement, §1 hazard table, §7 non-traction scope flag, §8 consolidated register, fail-state gap, rollaway risk (H2), and RF jamming/spoofing risk (H3).
- **v0.4**: added §0 (explicit safety scoping statement), §1 (hazard ID table supporting risk graph parameters), §7 (non-traction actuator scope flag), §8 (consolidated open-items register, replacing three overlapping lists in v0.3); added fail-state definition gap (steering on E-stop), rollaway risk (H2), RF jamming/spoofing risk (H3) as new open items; named Zephyr's specific IEC 61508 SEooC route-3s status in §5 rather than a general reference link.
- **v0.3**: prior version (component table, PLc draft calculation, phased rollout).
