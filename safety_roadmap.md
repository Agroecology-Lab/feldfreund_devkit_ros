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
| /estop/soft (software) | DONE | driver-level, devkit_driver, supplemental to formal safety system |
| /estop/front, /estop/back (hardware state topics) | DONE | driver-level |
| Bumper topics (front_top, front_bottom, back) | DONE | Not used in formal safety system |
| Physical hard-wired E-stop | Work in progress | Spec above |
| Bumper e-stops (physical) | Work in progress | Spec above |
| E-stop/bumper output stage (power cutoff) | Work in progress| Spec above |
| Wireless failsafe pendant | Work in progress | [Indus 1S transmitter](https://telemandosybaterias.com/en/p/indus-1s-868mhz-tyro-remotes-e-stop-wireless), €725, plus [Gemini 1S receiver](https://telemandosybaterias.com/en/p/gemini-230vac-tyro-remotes-e-stop-wireless), €771 (both excl. VAT, excl. delivery), €1,496 for the pair. Manufacturer claims PL-c, not yet checked against their declaration of conformity |
| Reversing alarm and flashing LED, motion-active | TO DO | [Brigade self-adjusting white sound reversing alarm](https://brigade-electronics.com/warning-systems/reversing-and-warning-alarms/self-adjusting-white-sound-reversing-alarms/), ambient-adjusting, integrated LED. Wire to motion state generally, not just reverse. 24V on safety circuit. This supports the avoidance assumption in the risk graph below, it is not part of the stop function itself |
| First-run terminal acceptance of E-stop / safety warning and disclaimer | DONE | prompted by manage.py during .env setup |
| Resume confirmation after a stop | Work in progress | After safety circuit triggered requires manual re-arm |

## Safety circuit components

| Supplier | Model | Notes |
|---|---|---|
| [Tapeswitch Corp.](https://www.tapeswitch.com/bumpers.html) | VBL, SE-45D, SE-75D, custom SE-C series | **Selected.** 4-wire configuration with PRSU/2 controller, Category 3. Datasheets and quote request forms |

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
**Risk Graph Parameters:** S2 (Severe/irreversible injury), F2 (Frequent/continuous exposure), P1 (Avoidance possible via white-sound alarm and flashing beacon). Yields a target of **PLc**. *Note: If $P1$ is dropped to $P2$ due to ambient noise or blind spots, the target escalates to **PLd**.*

---

### 1. In-Scope Safety Function Components

The whole-vehicle emergency motor-stop function forms a series safety loop containing:

1. **Physical E-Stops:** Dual Schneider XALK178 mushroom buttons ($2\times\text{NC}$ contacts in series).
2. **Wireless Failsafe Pendant:** Tyro Indus 1S transmitter + Gemini 1S receiver ($2\times\text{force-guided}$ safety relays).
3. **Physical Bumper:** Tapeswitch VBL Series (4-wire fail-safe loop, configured for overtravel cushioning).
4. **Safety Interface Logic:** Tapeswitch PRSU/2 Control Unit (dual force-operated NO safety relays).
5. **Main Power Actuators:** Dual Albright SW180 24V contactors (main contacts wired in series on the 48V $B+$ traction bus) equipped with TVS diode flyback suppression networks.

*Out of scope for safety calculations:* ROS 2 software nodes (`sentor_node`, `/estop/soft`), ESP32 motor controller drivers, reversing alarms, and high-level vision perception.

---

### 2. Safety Architecture & Category Assessment

* **Architecture Category:** **Category 3** (Dual-channel redundant structure across inputs, logic, and output power interlocks).
* **Diagnostic Coverage ($DC_{\text{avg}}$):** **Low ($60\%\text{--}90\%$)**, achieved by wiring the Normally Closed (NC) auxiliary microswitch contacts of both SW180 contactors in series into the PRSU/2 External Device Monitoring (EDM) reset loop.
* **Common Cause Failure (CCF):** Annex F scoring applies ($\ge 65$ points required). Achieved via channel isolation, overvoltage protection, and physical wiring separation.
* **Architectural Ceiling:** **PLd** (or **PLe** dependent on final $DC$ and $MTTF_d$ values), fully satisfying the baseline PLc target.

---

### 3. Combination Rule & Series Integrity

The total $PFH_d$ (Probability of Dangerous Failure per Hour) and PL ceiling are dictated by the worst-performing element in the series chain:

$$\text{Whole-Vehicle Stop Function} = \text{E-Stops} \longrightarrow \text{Gemini 1S} \longrightarrow \text{VBL Bumper / PRSU/2} \longrightarrow \text{Dual SW180s}$$

No individual component in this chain can have a rating lower than the overall target (PLc or PLd).

---

### 4. Outstanding Data Requirements (Pre-SISTEMA Verification)

To convert this target architecture into a fully verified safety calculation, the following quantitative inputs must be compiled:

1. **Tapeswitch VBL + PRSU/2 Metrics:** Obtain official $B_{10d}$, $MTTF_d$, and $PFH_d$ data from Tapeswitch Corp. for the **4-wire fail-safe** VBL bumper configuration paired with the PRSU/2 module.
2. **Tyro Gemini 1S EC Type-Examination:** Confirm $PFH_d$ figures and operational limits against the manufacturer's official Declaration of Conformity for the Indus 1S / Gemini 1S pair.
3. **SW180 Contactor Justification:** Document the $B_{10d}$ operations count for the Albright SW180 contactors under expected traction switching loads and record well-tried component status per ISO 13849-2.
4. **CCF Scoring Checklist:** Complete the formal ISO 13849-1 Annex F evaluation sheet to document a score $\ge 65$.
5. **SISTEMA Execution:** Run the final parameter set ($B_{10d}$, $MTTF_d$, $DC_{\text{avg}}$, CCF score) through SISTEMA (or equivalent IFA software) to output the finalized $PFH_d$ value for the vehicle.

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
