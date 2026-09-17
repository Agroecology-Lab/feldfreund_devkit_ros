# Sowbot Safety Roadmap, v0.3

## 1. E-stop and safety hardware

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

# Safety Loop Components

## Core

| # | Component | Model / Supplier | Role in loop | Key spec | Status | Data still needed |
|---|---|---|---|---|---|---|
| 1 | Physical E-stops | Schneider XALK178 ×2, [Kempston Controls](https://www.kempstoncontrols.co.uk/XALK178/Schneider/sku/479749), £29.05 each excl. VAT | Input (series NC) | 2×NC contacts | WIP | None |
| 2 | Wireless failsafe pendant | [Tyro Indus 1S transmitter](https://telemandosybaterias.com/en/p/indus-1s-868mhz-tyro-remotes-e-stop-wireless), €725, + [Gemini 1S receiver](https://telemandosybaterias.com/en/p/gemini-230vac-tyro-remotes-e-stop-wireless), €771, both excl. VAT and delivery | Input | 868MHz | WIP | PFHd vs. manufacturer's DoC (PL-c claim unverified) |
| 3 | Output contactors | Albright SW180 24V ×2 (series, 48V B+ bus), [Arc Components](https://www.arc-components.com/sw180-3-albright-single-acting-solenoid-contactor-24v-intermittent.html), £74.69 each excl. VAT. [2180-796 auxiliary micro-switch kit](https://www.arc-components.com/auxiliary-micro-switches-for-albright-contactors.html) (fits SW180/SW182), £32.09 excl. VAT | Output | 200A cont/400A peak, magnetic blowout, silver alloy contacts, TVS suppressors | WIP | B10d under traction load |
| 4 | IDEM GLM rope pull switch (note: this is "Mini Duty", not Heavy Duty — GLHL/GLHR is IDEM's Heavy Duty line) | [IDEM 143052 GLM 2NC 2NO M20](https://www.seltec.co.uk/products/idem-glm-guardian-line-mini-duty-rope-switch.html), £77.92 excl. VAT (£93.50 incl. VAT), Seltec | Input | Die-cast, up to 30–50m rope span, 2NC/2NO | TO DO | Confirm rope length needed; confirm Mini vs Heavy Duty intent; B10d/MTTFd/PFHd |
| **Total** | | **£1,919.66 ** (excl. VAT total was £1,599.56) | | | | |

## Supplemental

| # | Component | Model / Supplier | Role in loop | Key spec | Status | Data still needed |
|---|---|---|---|---|---|---|
| 1 | Relays / safety logic | [Tapeswitch PRSU/2](https://www.tapeswitch.com/store/products.php?cat=Interface+Controllers), $315.00 from Tapeswitch's own store | Logic | Cat 3, PL-e, TÜV-assessed, <30ms response; 2×N.O. positive-guided safety relays, AgSnO2 contacts, 250VAC/24VDC, 6A individual/13.8A combined | Selected | None |
| 2 | Bumper | [Tapeswitch VBL](https://www.tapeswitch.com/bumpers.html) (SE-45D/SE-75D/custom SE-C), quote-only, no fixed web price | Input | 4-wire fail-safe loop | Selected | B10d, MTTFd, PFHd from Tapeswitch |
| 3 | Reversing alarm/beacon | [Brigade SA-BBS-97](https://www.beaconsandlightbars.co.uk/product/brigade-electronics-brigade-sa-bbs-97-77-97db-smart-bbs-tek-white-sound-reversing-alarm-pn-sa-bbs-9-17914), £95, + [rotating LED ~£40](https://www.compass24.com/led-3600-rotating-beacon-flat-396940/black) | Not in stop function, avoidance measure only | 24V, wire to motion state generally not just reverse | TO DO | None |
| **Total** | | **£135.00** + **$315.00** (mixed currency; Tapeswitch VBL bumper excluded — quote-only, no fixed price) | | | | |

### Software and control status

| Item | Status | Notes |
|---|---|---|
| /estop/soft (software) | DONE | driver-level, devkit_driver, supplemental to formal safety system |
| /estop/front, /estop/back (hardware state topics) | DONE | driver-level |
| Bumper topics (front_top, front_bottom, back) | DONE | Not used in formal safety system |
| First-run terminal acceptance of E-stop / safety warning and disclaimer | DONE | prompted by manage.py during .env setup |
| Resume confirmation after a stop | WIP | after safety circuit triggered, requires manual re-arm |

Hardware build status (physical E-stop, bumper e-stops, output stage, wireless pendant) is tracked in the component table above, not repeated here.

## 2. Monitoring

| Item | Status | Notes |
|---|---|---|
| sentor (sowbot_monitor.yaml) | WIP, ~75% | monitors e-stop, bumpers, battery, camera, odom, neo heartbeat |
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

## Functional Safety Calculation (Draft)

**Target:** ISO 13849-1 Performance Level c (PLc) / Performance Level d (PLd)
**Risk Graph Parameters:** S2 (Severe/irreversible injury), F2 (Frequent/continuous exposure), P1 (Avoidance possible via white-sound alarm and flashing beacon). Yields a target of **PLc**. Note: if P1 is dropped to P2 due to ambient noise or blind spots, the target escalates to **PLd**.

### 1. In-scope safety function components

The whole-vehicle emergency motor-stop function is a series safety loop made of components 1–5 in the table above (physical E-stops, wireless pendant, bumper, PRSU/2 logic, SW180 contactors).

Out of scope for safety calculations: ROS 2 software nodes (sentor_node, /estop/soft), ESP32 motor controller drivers, reversing alarms, and high-level vision perception.

### 2. Safety architecture and category assessment

- **Architecture Category:** Category 3 (dual-channel redundant structure across inputs, logic, and output power interlocks).
- **Diagnostic Coverage (DCavg):** Low (60–90%), achieved by wiring the NC auxiliary microswitch contacts of both SW180 contactors in series into the PRSU/2 External Device Monitoring (EDM) reset loop.
- **Common Cause Failure (CCF):** Annex F scoring applies (≥65 points required). Achieved via channel isolation, overvoltage protection, and physical wiring separation.
- **Architectural ceiling:** PLd (or PLe depending on final DC and MTTFd values), which satisfies the baseline PLc target.

### 3. Combination rule and series integrity

Total PFHd and PL ceiling are set by the worst-performing element in the series chain:

E-Stops → Gemini 1S → VBL Bumper/PRSU/2 → Dual SW180s

No individual component in this chain can have a rating lower than the overall target (PLc or PLd).

### 4. Outstanding data requirements (pre-SISTEMA verification)

1. Tapeswitch VBL + PRSU/2 metrics: obtain official B10d, MTTFd, and PFHd data from Tapeswitch for the 4-wire fail-safe VBL bumper configuration paired with the PRSU/2 module.
2. Tyro Gemini 1S EC type-examination: confirm PFHd figures and operational limits against the manufacturer's Declaration of Conformity for the Indus 1S / Gemini 1S pair.
3. SW180 contactor justification: document the B10d operations count for the Albright SW180 contactors under expected traction switching loads, and record well-tried component status per ISO 13849-2.
4. CCF scoring checklist: complete the formal ISO 13849-1 Annex F evaluation sheet to a score ≥65.
5. SISTEMA execution: run the final parameter set (B10d, MTTFd, DCavg, CCF score) through SISTEMA (or equivalent IFA software) to output the finalised PFHd value for the vehicle.

### Phase 1: dev platform (current focus)

Audience: university labs, ag-tech researchers, software startups.

| Item | Status |
|---|---|
| PLc calculation | Draft in progress, see above |
| Third-party audit | NOT DONE |
| Compliance claim in docs or marketing | NONE, correctly |
| Standards used as design reference | YES, informal |
| Liability position | User's own risk, standard for a research/dev kit, stated in README |

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
