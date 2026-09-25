# Sowbot Safety Roadmap, v0.6.3

**Document control**
- Previous version: v0.6.2
- Status: living document, Phase 1 (dev platform)
- Scope: whole-vehicle emergency motor-stop function, ISO 13849-1. Does not cover implement, PTO, or manipulator safety, see §7.
- Owner: TBD (assign)

---

## 0. Safety scoping

The formal safety function is the hardwired 24V E-stop loop (§2, Core components) and the 48V traction interlock it drives. Everything else in this document (ROS 2 nodes, `sentor`, ESP32/STM32H7 firmware, perception, reversing alarm) is supervisory or defence-in-depth, **not** part of the rated safety function, and does not enter the PLc/PLd calculation in §6.

This split holds regardless of firmware changes (ESP32 to STM32H7, Lizard to ArduPilot CoginiPilot, RTOS choice). Moving the controller firmware does not move the safety boundary. 

---

## 1. Hazard identification

Supports the S2/F2/P1 risk graph parameters in §6. Not exhaustive, add rows as identified.

| ID | Hazard | Cause | Exposure | Current mitigation | Residual risk |
|---|---|---|---|---|---|
| H1 | Crush/impact from moving vehicle | Software fault, sensor failure, operator error | Continuous during field operation (F2) | Physical bumper + E-stop loop | Response time <30ms; detection envelope not yet sized against ISO 3691-4 |
| H2 | Rollaway after stop | Stop on slope, no parking brake | Not yet assessed (CONFIRM operating slope range) | Worm gear drive (assumed 40:1) self-locks tracks when unpowered | OPEN, self-locking ratio assumed, not yet confirmed against gearbox datasheet; no positive parking brake in BOM as backup |
| H3 | Wireless pendant jamming/spoofing | 868MHz interference or malicious signal | Low to Medium, unassessed | Tiered reaction-time/retry scheme (see §2 datasheet review notes) is a plausible packet-loss mitigation, not confirmed as such by Cattron | OPEN, no RF integrity requirement stated; **fail-state on total signal loss still undocumented (see O20)** |
| H4 | Undetected E-stop hardware degradation | Contactor welding, relay failure over time | Continuous | PRSU/2 EDM loop via SW180 aux microswitches | Covered by DCavg in §6, pending final CCF/SISTEMA figures |
| H5 | False negative on human detection | Radar detection not yet fitted or validated; visual perception excluded from safety calc | Continuous once deployed near people | Inxpect S101A radar (front and rear) with C203A control unit selected (see §2 Supplemental, §4); supervisory only per §0 until O24 is decided | OPEN, radar not yet built or tested; suitability for a moving platform unconfirmed (see O23) |

---

## 2. E-stop and safety hardware

### 24V SAFETY CONTROL LOOP

```
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
       │ (24v E-Stop Safe Power Feed)
       │
┌──────┴──────────────────────────────────────┐
│  Inxpect C203A Control Unit                 │  (Safety Controller - SIL 2 / PL d)
└──────┬──────────────────────────────────────┘
       │   ▲                      ▲
       │   │                      │
       │   │ (4-Wire Direct)      │ (M12 CAN Bus)
       │   │                      │
       │ ┌─┴──────────────────┐ ┌─┴──────────────────┐
       │ │ ASO Sentir Bumpers │ │ Inxpect S101A      │
       │ │ (x2)               │ │ Radars (x2)        │
       │ └────────────────────┘ └────────────────────┘
       │
       │ 
       │
       ├──────────────────────┐
       │                      │
┌──────┴───────────┐   ┌──────┴───────────┐
│ SW180 #1 Coil    │   │ SW180 #2 Coil    │  (24V DC Actuation Coils)
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

### Core - Pl c

| # | Component | Model / Supplier | Role in loop | Key spec | Status |
|---|---|---|---|---|---|
| 1 | Physical E-stops | Schneider XALK178 ×2, [Kempston Controls](https://www.kempstoncontrols.co.uk/XALK178/Schneider/sku/479749), £29.05 each excl. VAT | Input (series NC) | 2×NC contacts | WIP |
| 2 | Wireless failsafe pendant | [Tyro Indus 1S transmitter](https://telemandosybaterias.com/en/p/indus-1s-868mhz-tyro-remotes-e-stop-wireless), €725, + [Gemini 1S receiver](https://telemandosybaterias.com/en/p/gemini-230vac-tyro-remotes-e-stop-wireless), €771, both excl. VAT and delivery | Input | 868MHz, PL-c (EN-ISO 13849-1), EC type-examination certified (datasheets reviewed, see notes below) | WIP |
| 3 | Output contactors | Albright SW180 24V ×2 (series, 48V B+ bus), [Arc Components](https://www.arc-components.com/sw180-3-albright-single-acting-solenoid-contactor-24v-intermittent.html), £74.69 each excl. VAT. [2180-796 auxiliary micro-switch kit](https://www.arc-components.com/auxiliary-micro-switches-for-albright-contactors.html) (fits SW180/SW182), £32.09 excl. VAT | Output | 200A cont/400A peak, magnetic blowout, silver alloy contacts, TVS suppressors | WIP |
| 4 | IDEM GLM wire rope tether pull switch | [IDEM 143052 GLM 2NC 2NO M20](https://www.seltec.co.uk/products/idem-glm-guardian-line-mini-duty-rope-switch.html), 
| 3 | Reversing alarm/beacon | [Brigade SA-BBS-97](https://www.beaconsandlightbars.co.uk/product/brigade-electronics-brigade-sa-bbs-97-77-97db-smart-bbs-tek-white-sound-reversing-alarm-pn-sa-bbs-9-17914), £95, + [rotating LED ~£40](https://www.compass24.com/led-3600-rotating-beacon-flat-396940/black) | Not in stop function, avoidance measure only | 24V, wire to motion state generally, not just reverse | TO DO |
£77.92 excl. VAT (£93.50 incl. VAT), Seltec | Input | Die-cast, up to 30 to 50m rope span, 2NC/2NO | TO DO |
| **Total** | | **£1,919.66** (excl. VAT total was £1,599.56) | | | |

### Supplemental - PL d 

| # | Component | Model / Supplier | Role in loop | Key spec | Status |
|---|---|---|---|---|---|
| 2 | Bumper | [ASO Sentir](https://www.automation24.co.uk/safety-bumper-aso-sentir-1701-114-60-100-l4-0?orderCode=135)|4BY|2000| Input | 4-wire fail-safe loop | Selected. Wired into Inxpect C203A below [2006/42/EG](https://asosafety.com/en/produkt/sentir-bumper-100-120-l-o-2/), EN ISO 13856-3:2013, 2011/65/EU. |
| 4 | Human-detection radar sensors (front and rear) | Inxpect S101A ×2 (Fortop code IT100006, Inxpect 90202011), [Fortop UK](https://shop.fortop.co.uk/en/en/inxpect-it100006-s101a-ul-radar-sensor-90202011.html), £510.00 each (£1,020.00 for two; VAT status not stated on the page), delivery approx. 4 weeks | Input (human detection). Supervisory per §0, not in the §6 calculation unless O24 changes that | 24 GHz FMCW radar, SIL 2 / PL d (Inxpect); 0 to 4 m range, min. set distance 1 m; FOV 110°×30° (wide) or 50°×15° (narrow); max target speed 1.6 m/s; IP67; −30 to +60 °C; 12 V dc through control unit, 1.5 W; M12 connectors, CAN | Selected. Replaces the thermal detection plan. Suitability for a moving vehicle unconfirmed (O23). |
| 5 | Radar control unit | Inxpect C203A ×1 (Fortop code IT100024, Inxpect 90304011), [Fortop UK](https://shop.fortop.co.uk/en/en/c203a-ul-control-unit-200-series-it100024-90304011.html), £570.00 (VAT status not stated), 1 in stock | Logic (radar) | Connects up to 6 sensors; digital inputs and safety outputs; USB configuration via Inxpect Safety Application | Selected. PLd check (O23). |
| **Total** | | **£1,725.00** (£135.00 alarm/beacon + £1,590.00 Inxpect radar, Fortop prices, VAT status unstated) + **$315.00** (mixed currency;, quote-only, no fixed price) | | | |

Component-level data gaps (B10d, MTTFd, PFHd, rope length, etc.) are tracked once, in the register at §8, not repeated here.

### Wireless pendant — datasheet review notes

Both Indus 1S and Gemini 1S manufacturer datasheets (Tyro Remotes) have been reviewed in full. Findings:

- **Reaction time mismatch between transmitter and receiver documentation.** Indus 1S datasheet lists reaction time as 0.5 / 1.0 / 1.5 seconds (three tiers). Gemini 1S datasheet lists 0.5 / 1.0 / 1.5 / **2.0** seconds (four tiers). Not yet clear which figure governs system-level response time, or whether the fourth tier is a receiver-side timeout with no transmitter equivalent, or the retry/repeat mechanism that mitigates dropped packets on the shared 868MHz ISM band referenced in H3. Needs clarification from Cattron (see O21).
- **Internal inconsistency in the Indus 1S datasheet itself**: the front-page bullet states "Battery life uninterrupted use: 40 hours," while the technical specifications table on the same document states "Battery life: Approx. 50 hours." Not safety-critical, but worth flagging to Cattron as a documentation quality issue; treat the lower figure (40hr) as the conservative assumption until clarified.
- **IP rating discrepancy**: Cattron's website product page lists the Gemini 1S as IP66; the manufacturer datasheet lists it as **IP65**. Datasheet takes precedence — use IP65 in any enclosure/environmental rating decisions.
- Gemini 1S max current load: 4A per relay output. Confirm this is wired into PRSU/2 logic inputs only, not expected to switch contactor coil current directly (PRSU/2 output stage is rated 6A individual / 13.8A combined, see §2 Supplemental component 1).
- Confirmed: EC type-examination certification is referenced in the Indus 1S datasheet text. The certificate/DoC itself is a separate document, not yet obtained, and neither datasheet publishes PFHd or B10d figures.
- **Not documented in either datasheet, and not found anywhere public: the Gemini 1S's fail-state on total signal loss** (fails to commanded stop vs. holds last state). This is the single fact the H3/O3 risk assessment and the SISTEMA input both depend on. See O20.

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
| Aggregation layer + `/safety/level` | TO DO | see architecture note below |

Per §0: `sentor` and the software E-stop topics are diagnostic/supervisory. They are not part of the rated safety function.

**Architecture note (new):** `sentor`'s two heartbeats are a flat AND across ~12 monitors, with no ordered "how bad is it" signal and no de-escalation rule beyond the E-stop re-arm (O13). Fix: keep `sentor` as the raw topic/rate watcher, feed it into `diagnostic_aggregator` (mature, worst-child-wins tree aggregation) rather than `sentor`'s own `RobotStateMachine`/`sentor_guard` integration path — that path was added, reverted, and removed from sentor's own upstream (Dec 2025), so it's not stable enough to depend on. A small bridge node turns the aggregated status into one ordered `/safety/level` (`NOMINAL`→`DEGRADED`→`SAFE_STOP`) for the mission executor and Nav2 lifecycle to threshold on, instead of each wiring to raw E-stop/bumper/liveliness topics. Independent of this: the real-time `cmd_vel` gate (`nav2_collision_monitor`) stays under §5 and doesn't depend on this decision.

---

## 4. Perception

| Item | Status | Notes |
|---|---|---|
| Radar human detection (Inxpect S101A ×2 front/rear + C203A) | TO DO | selected, not yet built or tested; replaces the earlier thermal plan (MLX90640/MLX90614); see §2 Supplemental and H5; supervisory per §0 until O24 |
| Livestock false-positive tolerance | AGREED | fine for the detector to stop on livestock, safe default (agreed under the thermal plan, carried over to radar) |

---

## 5. Controller path

| Item | Status | Notes |
|---|---|---|
| ESP32 + Lizard DSL | DONE, current | hard real-time, but ESP-IDF quality |
| STM32H7 + ArduPilot Rover migration | NO | superseded by Cerebri path below |
| Cerebri on Zephyr & [FRDM-A-S32K358](https://www.nxp.com/design/design-center/development-boards-and-designs/FRDM-A-S32K358) dual 32-bit Arm® Cortex®-M7 cores operating in lockstep to support ASIL D functional safety | WIP | [Agroecology-Lab/cerebri](https://github.com/Agroecology-Lab/cerebri) |

Per §0: this layer is out of scope for the PLc calculation both before and after migration. The migration's value is defence-in-depth (EKF-based failsafes, geofencing) and long-run firmware certifiability. It does not change, and does not need to change, the §6 rating.

Zephyr's own safety programme (IEC 61508 SIL 3 SEooC, concept approval granted via route 3s, ISO 26262 alignment in progress) is the basis for the IEC 61508 reference above.

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
| ISO 21448 (SOTIF) | Safety of the intended functionality | Vision/radar degradation: mud, dust, glare, crop clutter (see H5) |

### Product classification

The devkit as currently shipped is **"partly completed machinery"** under both regimes tracked:
- **UK**: Supply of Machinery (Safety) Regulations 2008 (SI 2008/1597)
- **EU**: Machinery Directive 2006/42/EC (until 20 January 2027), then Machinery Regulation (EU) 2023/1230

This classification does **not** require a CE/UKCA mark or third-party certification at Phase 1, consistent with §9. It **does** require, before any unit ships: assembly instructions (Annex VI) and a Declaration of Incorporation (not a Declaration of Conformity) stating which essential health and safety requirements are met by the shipped components and that the unit must not be put into service until fully assembled. Track the Declaration of Incorporation as a deliverable, not yet in §8, add as O22.

The EU Machinery Regulation 2023/1230 replaces the Directive from 20 January 2027. If Phase 2/3 activity extends past that date for EU sales, documentation must be built against the Regulation (allows digital assembly instructions/Declaration of Incorporation, adds cybersecurity requirements relevant to H3, requires software update logging). No UK divergence announced yet; monitor.

### Functional safety calculation (draft)

**Target:** ISO 13849-1 Performance Level d (PLd)
**Risk graph parameters:** S2 (severe/irreversible injury, see H1), F2 (frequent/continuous exposure), P1 (avoidance possible via white-sound alarm and flashing beacon). Yields PLc. If P1 drops to P2 (ambient noise or blind spots), target escalates to PLd.

**In-scope components:** §2 Core and Supplemental tables (physical E-stops, wireless pendant, IDEM GLM tether switch, output contactors, PRSU/2 logic, bumper), per §0. The Inxpect PLd rated radar sensors and C203A control unit (§2 Supplemental 4 and 5) are excluded unless O24 brings them in.

**Architecture:**
- Category 3, dual-channel redundant structure across inputs, logic, and output power interlocks.
- Diagnostic coverage (DCavg): stated range 60 to 90% (Low), pending SW180 aux-microswitch EDM-loop verification. Not yet a fixed number, needs pinning down before the SISTEMA run, see §8.
- Common cause failure (CCF): Annex F scoring applies (≥65 points required); addressed via channel isolation, overvoltage protection, physical wiring separation. Score not yet computed, see §8.

**Combination rule:** total PFHd and PL ceiling are set by the worst-performing element in the series chain (E-Stops → Gemini 1S → IDEM GLM tether → VBL Bumper/PRSU/2 → dual SW180s). No component may rate below the overall target.

**Cybersecurity note:** the wireless pendant is an RF input to a safety function, sharing an unlicensed ISM band with other devices (H3). The tiered reaction-time/retry scheme found in the datasheet review is a plausible mitigation for dropped or collided packets but is not confirmed as such by Cattron, and critically, the receiver's behaviour when all retries fail (fail-state on total signal loss) is undocumented. This is now the single blocking question for both H3 and the wireless pendant's PL-d input to the SISTEMA calc, see O20.

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
| O3 | Tyro Gemini 1S / Indus 1S: confirm PFHd against manufacturer's Declaration of Conformity | §6 SISTEMA run, H3 | TO DO (PL-c claim currently unverified). **Datasheets for both units now reviewed in full — neither publishes PFHd/B10d. EC type-examination is referenced but the certificate itself has not been obtained.** |
| O4 | SW180 contactor: document B10d under expected traction switching loads; record well-tried component status (ISO 13849-2) | §6 SISTEMA run | TO DO |
| O5 | Complete formal ISO 13849-1 Annex F CCF checklist to ≥65 points | §6 SISTEMA run | TO DO |
| O6 | Run final parameter set through SISTEMA (or equivalent) for finalised PFHd | Phase 2 gate | TO DO (depends on O2 to O5, O20) |
| O7 | Resolve SW180 coil voltage against PRSU/2 output rating; decide single vs. redundant contactor | §2 Core, component 3 | WIP |
| O8 | Build and test physical hard-wired E-stop and bumper E-stops on current hardware | §2 | WIP |
| O9 | Check wireless pendant PL-c claim against declaration of conformity in practice (bench test) | §2 Core, component 2 | TO DO |
| O10 | Add reversing alarm and flashing LED, wired to motion state | §2 Supplemental, component 3 | TO DO |
| O11 | `sentor` hardware smoke test (currently sim-only) | §3 | TO DO |
| O12 | Add `ros2_medkit` black-box logging | §3 | TO DO |
| O13 | Decide resume-confirmation behaviour after E-stop trigger | §2 | WIP |
| O14 | Define fail-state for steering on E-stop trigger (currently undocumented) | §2 fail-state definition, H2 | OPEN |
| O15 | Assess rollaway risk on slope (H2): confirm 40:1 worm gear ratio and self-locking against gearbox datasheet; confirm operating slope range; decide if a positive parking brake is still needed as backup | §1 H2 | OPEN |
| O16 | Assess RF jamming/spoofing risk for wireless pendant (H3) | §6 cybersecurity note | OPEN |
| O17 | Human-detection hardware: Inxpect S101A ×2 (front/rear) + C203A selected and added to §2 Supplemental (components 4 and 5), replacing the earlier thermal plan (MLX90640/MLX90614); order, install and validate (H5) | §4, §1 H5 | TO DO |
| O18 | Track Zephyr IEC 61508 SEooC certification status directly rather than as an open-ended reference | §5 | Ongoing |
| O19 | IDEM GLM wire rope tether: confirm rope length needed; obtain B10d/MTTFd/PFHd | §6 SISTEMA run, §2 Core component 4 | TO DO |
| O20 | Obtain written confirmation from Cattron of the Gemini 1S's fail-state on total signal loss (fails to commanded stop vs. holds last state) | H3, O3, §6 SISTEMA run | OPEN — blocking |
| O21 | Clarify with Cattron which reaction-time figure governs system response: Indus 1S's 3-tier (0.5/1.0/1.5s) vs Gemini 1S's 4-tier (0.5/1.0/1.5/2.0s) spec | §1 H1, §6 SISTEMA run | OPEN |
| O22 | Prepare Declaration of Incorporation and assembly instructions (Annex VI) for partly-completed-machinery shipment, per §6 Product classification | Any unit shipment | TO DO |
| O23 | Confirm with Inxpect / Fortop: (a) S101A is approved for mounting on a moving vehicle and detects a stationary person ahead of a moving platform; (b) S101A pairs with C203A; (c) sensor-to-output response time against H1; (d) outdoor use; (e) C203A supply voltage against the 24V safety power | §1 H5, §2 Supplemental 4 and 5 | OPEN |
| O24 | Decide whether the radar output stays supervisory (ROS layer only, per §0) or is wired into the PRSU/2 stop loop. If the latter, re-open §0 and re-run §6 | §0, §6 | OPEN |
| O25 | **(new)** Adopt `diagnostic_aggregator` between `sentor`'s per-topic output and a single ordered `/safety/level`, replacing the flat `/safety/heartbeat` + `/warning/heartbeat` pair | §3 | TO DO |
| O26 | **(new)** Define per-monitor de-escalation policy (auto-clear vs. requires manual re-arm), extending O13's E-stop-specific rule to every monitor feeding `/safety/level` | §3, O13 | TO DO |
| O27 | **(new)** Wire mission executor and Nav2 lifecycle to threshold on `/safety/level` rather than each subscribing to raw E-stop/bumper/node-liveliness topics individually | §3 | TO DO |

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
| SOTIF assessment for vision/radar degradation cases | TO DO |
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

- **v0.7**: added §3 supervisory architecture note — adopting `diagnostic_aggregator` for level aggregation over the `RobotStateMachine`/`sentor_guard` path (found unstable in sentor's own upstream history, Dec 2025); added O25–O27 for the aggregation layer, per-level de-escalation policy, and wiring the mission executor/Nav2 to a new `/safety/level` topic.
- **v0.6.2**: added Inxpect S101A ×2 (front and rear) and one C203A control unit, both from Fortop UK, to §2 Supplemental as components 4 and 5 (£510.00 each and £570.00) and updated the Supplemental total; replaced the thermal detection plan (MLX90640/MLX90614) with the radar in H5, §4, the §6 SOTIF row, §9 Phase 2 and O17; excluded the radar from the §6 in-scope list pending O24; added O23 (application and compatibility checks with Inxpect/Fortop) and O24 (whether the radar output stays supervisory or enters the stop loop, which would re-open §0 and §6).
- **v0.6.1**: added wireless pendant datasheet review notes to §2 (reaction-time tier mismatch between Indus 1S and Gemini 1S, internal battery-life inconsistency in the Indus 1S sheet, IP66-vs-IP65 discrepancy between website and datasheet); flagged that Tapeswitch's public PSSR-2 product must not be conflated with PRSU/2 (§2 Supplemental, O2); updated H3 and the §6 cybersecurity note with the fail-state-on-signal-loss gap; added new open items O20 (Gemini 1S fail-state confirmation, blocking), O21 (reaction-time reconciliation), O22 (Declaration of Incorporation / assembly instructions); added §6 Product classification subsection covering UK Supply of Machinery (Safety) Regulations 2008, EU Machinery Directive 2006/42/EC, and the 20 January 2027 transition to EU Machinery Regulation (EU) 2023/1230.
- **v0.6**: updated H2 and O15 with the drivetrain finding that the worm gear self-locks the tracks when unpowered, assumed ratio 40:1 pending confirmation against the actual gearbox datasheet. H2 stays OPEN until that's confirmed and a decision is made on whether a positive parking brake is still needed as backup.
- **v0.5**: restored the v0.3 Core/Supplemental table split in §2, including the IDEM GLM wire rope tether switch (Core, component 4) that v0.4 dropped when it merged the two tables into one; added the tether switch back as O19 in the open items register. Kept v0.4's §0 safety scoping statement, §1 hazard table, §7 non-traction scope flag, §8 consolidated register, fail-state gap, rollaway risk (H2), and RF jamming/spoofing risk (H3).
- **v0.4**: added §0 (explicit safety scoping statement), §1 (hazard ID table supporting risk graph parameters), §7 (non-traction actuator scope flag), §8 (consolidated open-items register, replacing three overlapping lists in v0.3); added fail-state definition gap (steering on E-stop), rollaway risk (H2), RF jamming/spoofing risk (H3) as new open items; named Zephyr's specific IEC 61508 SEooC route-3s status in §5 rather than a general reference link.
- **v0.3**: prior version (component table, PLc draft calculation, phased rollout).
