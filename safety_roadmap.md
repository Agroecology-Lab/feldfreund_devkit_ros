# Sowbot Safety Roadmap, v0.6

**Document control**
- Previous version: v0.5
- Status: living document, Phase 1 (dev platform)
- Scope: whole-vehicle emergency motor-stop function, ISO 13849-1. Does not cover implement, PTO, or manipulator safety, see §7.
- Owner: TBD (assign)

---

## 0. Safety scoping

The formal safety function is the hardwired 24V E-stop loop (§2, Core components) and the 48V traction interlock it drives. Everything else in this document (ROS 2 nodes, `sentor`, ESP32/STM32H7 firmware, perception, reversing alarm) is supervisory or defence-in-depth, **not** part of the rated safety function, and does not enter the PLc/PLd calculation in §6.

This split holds regardless of firmware changes (ESP32 to STM32H7, Lizard to ArduPilot, RTOS choice). Moving the controller firmware does not move the safety boundary. Any future change that would make software part of the certified stop path requires re-opening this section and re-running the calculation in §6, not a note added elsewhere.

---

## 1. Hazard identification

Supports the S2/F2/P1 risk graph parameters in §6. Not exhaustive, add rows as identified.

| ID | Hazard | Cause | Exposure | Current mitigation | Residual risk |
|---|---|---|---|---|---|
| H1 | Crush/impact from moving vehicle | Software fault, sensor failure, operator error | Continuous during field operation (F2) | Physical bumper (VBL) + E-stop loop | Response time <30ms (PRSU/2 spec); detection envelope not yet sized against ISO 3691-4 |
| H2 | Rollaway after stop | Stop on slope, no parking brake | Not yet assessed (CONFIRM operating slope range) | Worm gear drive (assumed 40:1) self-locks tracks when unpowered | OPEN, self-locking ratio assumed, not yet confirmed against gearbox datasheet; no positive parking brake in BOM as backup |
| H3 | Wireless pendant jamming/spoofing | 868MHz interference or malicious signal | Low to Medium, unassessed | None identified | OPEN, no RF integrity requirement stated |
| H4 | Undetected E-stop hardware degradation | Contactor welding, relay failure over time | Continuous | PRSU/2 EDM loop via SW180 aux microswitches | Covered by DCavg in §6, pending final CCF/SISTEMA figures |
| H5 | False negative on human detection | Thermal sensor not yet fitted; visual perception excluded from safety calc | Continuous once deployed near people | None yet (see §4) | OPEN, thermal detection still TO DO |

---

## 2. E-stop and safety hardware

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

### Core

| # | Component | Model / Supplier | Role in loop | Key spec | Status |
|---|---|---|---|---|---|
| 1 | Physical E-stops | Schneider XALK178 ×2, [Kempston Controls](https://www.kempstoncontrols.co.uk/XALK178/Schneider/sku/479749), £29.05 each excl. VAT | Input (series NC) | 2×NC contacts | WIP |
| 2 | Wireless failsafe pendant | [Tyro Indus 1S transmitter](https://telemandosybaterias.com/en/p/indus-1s-868mhz-tyro-remotes-e-stop-wireless), €725, + [Gemini 1S receiver](https://telemandosybaterias.com/en/p/gemini-230vac-tyro-remotes-e-stop-wireless), €771, both excl. VAT and delivery | Input | 868MHz | WIP |
| 3 | Output contactors | Albright SW180 24V ×2 (series, 48V B+ bus), [Arc Components](https://www.arc-components.com/sw180-3-albright-single-acting-solenoid-contactor-24v-intermittent.html), £74.69 each excl. VAT. [2180-796 auxiliary micro-switch kit](https://www.arc-components.com/auxiliary-micro-switches-for-albright-contactors.html) (fits SW180/SW182), £32.09 excl. VAT | Output | 200A cont/400A peak, magnetic blowout, silver alloy contacts, TVS suppressors | WIP |
| 4 | IDEM GLM wire rope tether pull switch | [IDEM 143052 GLM 2NC 2NO M20](https://www.seltec.co.uk/products/idem-glm-guardian-line-mini-duty-rope-switch.html), £77.92 excl. VAT (£93.50 incl. VAT), Seltec | Input | Die-cast, up to 30 to 50m rope span, 2NC/2NO | TO DO |
| **Total** | | **£1,919.66** (excl. VAT total was £1,599.56) | | | |

### Supplemental

| # | Component | Model / Supplier | Role in loop | Key spec | Status |
|---|---|---|---|---|---|
| 1 | Relays / safety logic | [Tapeswitch PRSU/2](https://www.tapeswitch.com/store/products.php?cat=Interface+Controllers), $315.00 from Tapeswitch's own store | Logic | Cat 3, PL-e, TÜV-assessed, <30ms response; 2×N.O. positive-guided safety relays, AgSnO2 contacts, 250VAC/24VDC, 6A individual/13.8A combined | Selected |
| 2 | Bumper | [Tapeswitch VBL](https://www.tapeswitch.com/bumpers.html) (SE-45D/SE-75D/custom SE-C), quote-only, no fixed web price | Input | 4-wire fail-safe loop | Selected |
| 3 | Reversing alarm/beacon | [Brigade SA-BBS-97](https://www.beaconsandlightbars.co.uk/product/brigade-electronics-brigade-sa-bbs-97-77-97db-smart-bbs-tek-white-sound-reversing-alarm-pn-sa-bbs-9-17914), £95, + [rotating LED ~£40](https://www.compass24.com/led-3600-rotating-beacon-flat-396940/black) | Not in stop function, avoidance measure only | 24V, wire to motion state generally, not just reverse | TO DO |
| **Total** | | **£135.00** + **$315.00** (mixed currency; Tapeswitch VBL bumper excluded, quote-only, no fixed price) | | | |

Component-level data gaps (B10d, MTTFd, PFHd, rope length, etc.) are tracked once, in the register at §8, not repeated here.


### Software and control status

| Item | Status | Notes |
|---|---|---|
| `/estop/soft` (software) | DONE | driver-level, `devkit_driver`, supplemental to the formal safety system |
| `/estop/front`, `/estop/back` (hardware state topics) | DONE | driver-level |
| Bumper topics (`front_top`, `front_bottom`, `back`) | DONE | not used in formal safety system |
| First-run terminal acceptance of E-stop/safety warning and disclaimer | DONE | prompted by `manage.py` during `.env` setup |
| Resume confirmation after a stop | WIP | after safety circuit triggers, requires manual re-arm |

Hardware build status is tracked in the tables above and in §8, not repeated here.

---

## 3. Monitoring

| Item | Status | Notes |
|---|---|---|
| `sentor` (`sowbot_monitor.yaml`) | WIP, ~75% | monitors e-stop, bumpers, battery, camera, odom, neo heartbeat |
| `sentor_node.py` wired into `devkit.launch.py` | DONE | |
| `sentor` hardware smoke test | TO DO | validated in sim only so far |
| Battery voltage cutoff threshold | TO DO | marked `# TODO: CONFIRM` in `sowbot_monitor.yaml`, no value set |
| `ros2_medkit` black-box logging | TO DO | not in repo or dependency list yet |

Per §0: `sentor` and the software E-stop topics are diagnostic/supervisory. They are not part of the rated safety function.

---

## 4. Perception

| Item | Status | Notes |
|---|---|---|
| Thermal (MLX90640/MLX90614) human detection | TO DO | not in repo or BOM, see H5 |
| Livestock false-positive tolerance | AGREED | fine for thermal to stop on livestock, safe default |

---

## 5. Controller path

| Item | Status | Notes |
|---|---|---|
| ESP32 + Lizard DSL | DONE, current | hard real-time motor PID and bumper cutoff |
| STM32H7 + ArduPilot Rover migration | TO DO | EKF3, failsafes, geofencing, SITL testing, community scrutiny. Reference: [ArduPilot Zephyr HAL: Flying on a BeagleV-Fire](https://www.beagleboard.org/projects/ardupilot-on-zephyr-flying-on-the-beaglev-fire), [Zephyr Safety Overview](https://docs.zephyrproject.org/latest/safety/safety_overview.html) |

Per §0: this layer is out of scope for the PLc calculation both before and after migration. The migration's value is defence-in-depth (EKF-based failsafes, geofencing) and long-run firmware certifiability. It does not change, and does not need to change, the §6 rating.

Zephyr's own safety programme (IEC 61508 SIL 3 SEooC, concept approval granted via route 3s, ISO 26262 alignment in progress) is the basis for the IEC 61508 reference above. Track its certification status directly rather than treating this as open-ended.

---

## 6. Regulatory compliance

No compliance claimed. Reference standards only until formal assessment or audit is done.

**Target: PLc, ISO 13849-1, for the whole-vehicle stop function.**

| Standard | Domain | Relevance |
|---|---|---|
| ISO 18497 | Highly automated ag machinery | Hazard zones, operational modes for autonomous field work |
| ISO 25119 / AgPL | Tractor and ag electronics functional safety | AgPL target for motor-stop interlocks |
| ISO 13849 / PL | Machinery safety, control systems | PLc target for E-stop relay, bumper circuit |
| ISO 3691-4 | AGV obstacle detection | Clearance rules, braking distance, detection envelope sizing |
| IEC 61508 | Functional safety, E/E/PE systems | Reference for controller firmware architecture (§5) |
| ISO 21448 (SOTIF) | Safety of the intended functionality | Vision/thermal degradation: mud, dust, glare (see H5) |

### Functional safety calculation (draft)

**Target:** ISO 13849-1 Performance Level c (PLc) / Performance Level d (PLd)
**Risk graph parameters:** S2 (severe/irreversible injury, see H1), F2 (frequent/continuous exposure), P1 (avoidance possible via white-sound alarm and flashing beacon). Yields PLc. If P1 drops to P2 (ambient noise or blind spots), target escalates to PLd.

**In-scope components:** §2 Core and Supplemental tables (physical E-stops, wireless pendant, IDEM GLM tether switch, output contactors, PRSU/2 logic, bumper), per §0.

**Architecture:**
- Category 3, dual-channel redundant structure across inputs, logic, and output power interlocks.
- Diagnostic coverage (DCavg): stated range 60 to 90% (Low), pending SW180 aux-microswitch EDM-loop verification. Not yet a fixed number, needs pinning down before the SISTEMA run, see §8.
- Common cause failure (CCF): Annex F scoring applies (≥65 points required); addressed via channel isolation, overvoltage protection, physical wiring separation. Score not yet computed, see §8.
- Architectural ceiling: PLd (or PLe depending on final DC/MTTFd), satisfying the PLc baseline.

**Combination rule:** total PFHd and PL ceiling are set by the worst-performing element in the series chain (E-Stops → Gemini 1S → IDEM GLM tether → VBL Bumper/PRSU/2 → dual SW180s). No component may rate below the overall target.

**Cybersecurity note (OPEN, not previously addressed):** the wireless pendant is an RF input to a safety function. Jamming or spoofing risk (H3) is not currently assessed against the Tyro/Gemini declaration of conformity. Add to §8 before claiming PLc against this input.

---

## 7. Non-traction actuators (OPEN, scope flag)

This roadmap covers the traction/motor-stop function only. Any future actuator outside that scope, an implement, a lift, an arm, PTO-equivalent, has no hazard analysis or safety architecture defined here and must not be assumed covered by §2's rating. Add a dedicated section before any such actuator is fielded.

---

## 8. Open items register

Single tracked list. Supersedes the separate 'data still needed', 'outstanding data requirements', and 'build order' lists in v0.3, and restores the IDEM GLM tether item that dropped out when v0.4 merged the Core and Supplemental tables. Ordered roughly by build sequence, re-order as priorities shift.

| ID | Item | Blocks | Status |
|---|---|---|---|
| O1 | Confirm battery voltage cutoff threshold | `sowbot_monitor.yaml` completion | TO DO |
| O2 | Tapeswitch VBL + PRSU/2: obtain B10d, MTTFd, PFHd for the 4-wire fail-safe configuration | §6 SISTEMA run | TO DO |
| O3 | Tyro Gemini 1S: confirm PFHd against manufacturer's Declaration of Conformity | §6 SISTEMA run, H3 | TO DO (PL-c claim currently unverified) |
| O4 | SW180 contactor: document B10d under expected traction switching loads; record well-tried component status (ISO 13849-2) | §6 SISTEMA run | TO DO |
| O5 | Complete formal ISO 13849-1 Annex F CCF checklist to ≥65 points | §6 SISTEMA run | TO DO |
| O6 | Run final parameter set through SISTEMA (or equivalent) for finalised PFHd | Phase 2 gate | TO DO (depends on O2 to O5) |
| O7 | Resolve SW180 coil voltage against PRSU/2 output rating; decide single vs. redundant contactor | §2 Core, component 3 | WIP |
| O8 | Build and test physical hard-wired E-stop and bumper E-stops on current hardware | §2 | WIP |
| O9 | Check wireless pendant PL-c claim against declaration of conformity in practice (bench test) | §2 Core, component 2 | TO DO |
| O10 | Add reversing alarm and flashing LED, wired to motion state | §2 Supplemental, component 3 | TO DO |
| O11 | `sentor` hardware smoke test (currently sim-only) | §3 | TO DO |
| O12 | Add `ros2_medkit` black-box logging | §3 | TO DO |
| O13 | Decide resume-confirmation behaviour after E-stop trigger | §2 | WIP |
| O14 | Define fail-state for steering on E-stop trigger (currently undocumented) | §2 fail-state definition, H2 | OPEN (new) |
| O15 | Assess rollaway risk on slope (H2): confirm 40:1 worm gear ratio and self-locking against gearbox datasheet; confirm operating slope range; decide if a positive parking brake is still needed as backup | §1 H2 | OPEN |
| O16 | Assess RF jamming/spoofing risk for wireless pendant (H3) | §6 cybersecurity note | OPEN (new) |
| O17 | Thermal human-detection hardware selection and BOM entry (H5) | §4, §1 H5 | TO DO |
| O18 | Track Zephyr IEC 61508 SEooC certification status directly rather than as an open-ended reference | §5 | Ongoing |
| O19 | IDEM GLM wire rope tether: confirm rope length needed; obtain B10d/MTTFd/PFHd | §6 SISTEMA run, §2 Core component 4 | TO DO |

---

## 9. Phased rollout

### Phase 1: dev platform (current focus)
Audience: university labs, ag-tech researchers, software startups.

| Item | Status |
|---|---|
| PLc calculation | Draft in progress, §6 |
| Third-party audit | NOT DONE |
| Compliance claim in docs or marketing | NONE, correctly |
| Standards used as design reference | YES, informal |
| Liability position | User's own risk, stated in README |

No certification work needed at this phase. Reference the standards, do not claim them.

### Phase 2: OEM modular subsystems
Audience: startups integrating Sowbot's drive/safety core.

| Item | Status |
|---|---|
| PLc calculation for E-stop and bumper circuit | Draft in progress, gate for this phase |
| SOTIF assessment for vision/thermal degradation cases | TO DO |
| IEC 61508 architecture review | TO DO |
| Third-party certification | NOT REQUIRED YET; formal internal assessment is |

### Phase 3: commercial sale to farmers
Audience: commercial growers, farm management enterprises.

| Item | Status |
|---|---|
| Full ISO 18497 compliance | REQUIRED before sale |
| ISO 13849 PLc certification, or equivalent | REQUIRED before sale |
| Third-party safety audit | REQUIRED before sale |
| Field trial history | REQUIRED before sale |
| Insurance and liability structure | NOT YET SCOPED |

---

## Revision history

- **v0.6**: updated H2 and O15 with the drivetrain finding that the worm gear self-locks the tracks when unpowered, assumed ratio 40:1 pending confirmation against the actual gearbox datasheet. H2 stays OPEN until that's confirmed and a decision is made on whether a positive parking brake is still needed as backup.
- **v0.5**: restored the v0.3 Core/Supplemental table split in §2, including the IDEM GLM wire rope tether switch (Core, component 4) that v0.4 dropped when it merged the two tables into one; added the tether switch back as O19 in the open items register. Kept v0.4's §0 safety scoping statement, §1 hazard table, §7 non-traction scope flag, §8 consolidated register, fail-state gap, rollaway risk (H2), and RF jamming/spoofing risk (H3).
- **v0.4**: added §0 (explicit safety scoping statement), §1 (hazard ID table supporting risk graph parameters), §7 (non-traction actuator scope flag), §8 (consolidated open-items register, replacing three overlapping lists in v0.3); added fail-state definition gap (steering on E-stop), rollaway risk (H2), RF jamming/spoofing risk (H3) as new open items; named Zephyr's specific IEC 61508 SEooC route-3s status in §5 rather than a general reference link.
- **v0.3**: prior version (component table, PLc draft calculation, phased rollout).
