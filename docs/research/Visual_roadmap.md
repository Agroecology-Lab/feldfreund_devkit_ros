# Vision System Roadmap — Sowbot / Avaota A1
## `crop_row_node` · YOLOX-Nano Weed Detection · ROS 2

**Deployment targets:** UK outdoor lettuce · India young crop weeding
**Platform:** Allwinner T527 (8× A55, Zhouyi Z2 NPU @ 2 TOPS)

---

## 1. Current State Assessment

### What works and must be preserved

The `crop_row_node` architecture is sound and should not be restructured.
Specifically:

- The **Cherubini & Chaumette visual servoing interaction matrix** is
  mathematically correct for monocular row-following and well-suited to
  the slow-speed, low-inertia dynamics of a field weeding robot.
- The **ROS 2 node design** — AOC conditions on separate topics, heartbeat
  for `PERCEPTION_DEGRADED` (M10), service-gated `cmd_vel` — is clean,
  safe, and should be treated as fixed architecture.
- The **BSD-2 provenance chain** from Agricultural-Robotics-Bonn is
  properly tracked. Keep it.

None of the controller logic, topic structure, or service interface needs
to change at any point in this roadmap.

---

### The core fragility

The entire perception stack rests on one assumption:

> **Crop rows are the dominant source of green pixels in the image.**

For UK outdoor lettuce this assumption holds reasonably well under the
diffuse overcast light that dominates British growing conditions — which is
actually close to the ideal case for ExG. However it breaks down in four
conditions that will be encountered regularly:

- **Early crop stage** — lettuce at emergence is small and sparse; weeds
  may be denser and larger than the crop itself
- **Weed pressure** — weeds are ExG-positive by definition; the pipeline
  cannot distinguish crop green from weed green
- **Mud splash** — wet UK field conditions deposit soil on lower leaves,
  suppressing green channel values and fragmenting contours
- **Indian deployment** — stronger directional sunlight and greater
  illumination variation cause saturation and shadow artefacts that
  unnormalised ExG cannot handle

The first two conditions are universal to both deployments.
The last two are context-specific but both foreseeable.

---

### The architectural conflict

The two subsystems are currently in direct competition:

```
Camera frame
  │
  ├─► ExG mask → contours → row centre    (consumes ALL green pixels)
  │
  └─► YOLOX-Nano → weed bounding boxes   (identifies a subset of green pixels)
```

Weeds that YOLOX correctly detects are simultaneously being used by the
row-following pipeline as evidence of crop row position. Under heavy weed
pressure — precisely when weed detection matters most agronomically — the
row-following steers toward weeds. This must be resolved before the system
is considered reliable.

---

### Image topic architecture (decided)

Both `crop_row_node` and `weed_detect_node` subscribe to the same topic:

```
/caatinga_vision/row_nav/image_raw
```

Run both nodes as **composable nodes in a shared container** with
intra-process communication. The camera image is published once, serialised
once, and delivered as a shared pointer — zero copy between the two nodes
on the same SBC. No code changes to either node are required; the topic
name is identical in both.

```python
# launch/vision.launch.py
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

container = ComposableNodeContainer(
    name='vision_container',
    namespace='',
    package='rclcpp_components',
    executable='component_container',
    composable_node_descriptions=[
        ComposableNode(
            package='sowbot_vision',
            plugin='sowbot_vision::CameraSourceNode',
            name='camera_source_node',
        ),
        ComposableNode(
            package='sowbot_vision',
            plugin='sowbot_vision::CropRowNode',
            name='crop_row_node',
        ),
        ComposableNode(
            package='sowbot_vision',
            plugin='sowbot_vision::WeedDetectNode',
            name='weed_detect_node',
        ),
    ],
)
```

`cv_bridge.imgmsg_to_cv2()` in each node still performs its own `Mat`
allocation — at 640×480×3 this is ~1.8 MB peak simultaneous. This is
not a bottleneck at these resolutions and should not be optimised until
profiling demonstrates otherwise.

---

### Latency budget

At 30 Hz the end-to-end budget is **33 ms** from image capture to
`cmd_vel` publish. Estimated current pipeline (unverified —
profile on-device before any optimisation):

| Stage | Estimated |
|---|---|
| ROS 2 intra-process delivery | < 0.5 ms |
| `imgmsg_to_cv2` decode | 1–3 ms |
| ExG + Otsu + dilation (640×480) | 3–8 ms |
| `findContours` + centroids | 1–4 ms |
| `detect_crop_rows` (polyfit × 8 windows) | < 1 ms |
| `visual_servoing_ctl` | < 1 ms |
| **Total** | **~6–17 ms** |

There is margin. **Measure this first.** Every subsequent change must be
benchmarked against this baseline.

---

## 2. Immediate — Harden What Exists
### (No model training. No NPU SDK. No architectural changes.)

---

### M0 · Profile on-device (Week 1)

```bash
# Measure header-stamp → publish latency on the debug image
ros2 topic delay /caatinga_vision/row_nav/debug_image

# Per-message timing
ros2 run rqt_topic rqt_topic
```

Record: mean latency, p95 latency, CPU core utilisation (`htop`),
memory bandwidth (`sudo sar -b 1`). This is the baseline. No claim about
performance is valid without it.

---

### M1 · Normalised ExG (Week 1–2)

The current ExG is unnormalised. Replace the calculation in
`compute_exg_mask`:

```python
# BEFORE — unnormalised, saturates in bright sun
exg = 2 * g - r - b

# AFTER — normalised to [−1, 1], illumination-robust
img_f = bgr_img.astype("float32")
b, g, r = img_f[:,:,0], img_f[:,:,1], img_f[:,:,2]
total = r + g + b + 1e-6
exg = 2*(g/total) - (r/total) - (b/total)

# Scale to uint8 for Otsu (maps [−1,1] → [0,255])
exg_u8 = np.clip((exg + 1.0) * 127.5, 0, 255).astype("uint8")
```

**Why this matters for UK lettuce:** Even under consistent British overcast,
the robot will operate at golden hour (early morning slots to avoid
compaction on wet soil), under poly-tunnel edges, and in partial cloud
transitions. Normalised ExG is stable across all of these at zero compute
cost.

**Why this matters for India:** Strong directional sun causes channel
saturation. Normalised ExG recovers correctly where unnormalised ExG
collapses to zero contrast.

This is a one-function change, no retraining, no SDK. Do it first.

---

### M2 · Weed exclusion mask (Week 2–3)

Before ExG processing, zero out image regions that YOLOX-Nano has already
confirmed as weeds. This resolves the architectural conflict described in
section 1 at zero additional inference cost.

Add a second subscriber and a temporal synchroniser to `crop_row_node`:

```python
from message_filters import ApproximateTimeSynchronizer, Subscriber
from vision_msgs.msg import Detection2DArray

# Replace the single image subscriber with a synchronised pair
self._img_sub = Subscriber(self, Image, self.image_topic)
self._det_sub = Subscriber(self, Detection2DArray, "/weed_detections")
self._sync = ApproximateTimeSynchronizer(
    [self._img_sub, self._det_sub], queue_size=10, slop=0.05)
self._sync.registerCallback(self._on_image_and_detections)
```

Masking function — called before `compute_exg_mask`:

```python
def _mask_weed_regions(self, bgr: np.ndarray,
                        detections: Detection2DArray) -> np.ndarray:
    masked = bgr.copy()
    for det in detections.detections:
        bb = det.bbox
        x1 = max(0, int(bb.center.position.x - bb.size_x / 2))
        y1 = max(0, int(bb.center.position.y - bb.size_y / 2))
        x2 = min(self.img_w, int(bb.center.position.x + bb.size_x / 2))
        y2 = min(self.img_h, int(bb.center.position.y + bb.size_y / 2))
        masked[y1:y2, x1:x2] = 0
    return masked
```

The 50 ms `slop` tolerance accommodates YOLOX-Nano running at 10–15 Hz
while `crop_row_node` runs at 30 Hz. A false weed detection that masks a
crop cluster will degrade the row centre estimate — which is exactly why
M3 (YOLOX validation) must precede relying on these boxes in production.

---

### M2.1 · Parameterise the dilation kernel (Week 2–3)

The 10×10 dilation kernel is hard-coded for a specific crop density.
Lettuce at emergence has different spacing than lettuce at canopy closure,
and Indian crop rows may have different plant separation entirely.

Move to `crop_row_params.yaml`:

```yaml
crop_row_node:
  ros__parameters:
    # UK lettuce — adjust per crop stage
    dilation_kernel_size: 10    # px; increase at emergence, decrease at canopy
    min_contour_area: 10.0      # px²; increase to filter soil/mud splash noise
    n_scan_windows: 8
    window_width: 80
```

Add a merge step that groups contour centres within `dilation_kernel_size`
pixels of each other before passing to `detect_crop_rows`. This makes the
system tuneable across crop stages without code changes.

---

## 3. Short-Term — NPU Weed Detection Deployment
### (Weeks 3–8)

---

### M3 · Validate YOLOX-Nano on UK lettuce weeds (Week 3–5)

Before any NPU bounding boxes are trusted as exclusion masks (M2), verify
the model's performance on the actual target weed species. Common weeds in
UK outdoor lettuce:

- Annual meadow grass (*Poa annua*)
- Fat hen (*Chenopodium album*)
- Chickweed (*Stellaria media*)
- Groundsel (*Senecio vulgaris*)
- Shepherd's purse (*Capsella bursa-pastoris*)
- Field speedwell (*Veronica persica*)

Collect a minimum **300-image validation set** from the target fields at
multiple crop growth stages. Measure per-class precision and recall.

A false positive (crop detected as weed) that triggers the exclusion mask
directly corrupts the row-centre estimate. Set a **high confidence threshold
(≥ 0.55)** for the exclusion mask even if this reduces recall, because the
cost of a false positive here is a navigation error, not just a missed
detection.

For India, repeat this process locally before deployment — weed flora will
differ substantially from UK species.

---

### M4 · NPU inference latency benchmark (Week 4–6)

After completing the Zhouyi SDK setup (see companion deployment guide),
establish actual on-device timing before committing to any inference
frequency targets:

```bash
# Using the siengine runtime demo as the template
# (from AI610-SDK-r1p3-AIoT/siengine/nn-runtime-user-case-example)
./weed_detect_bench \
    --model yolox_nano_weed.bin \
    --input test_640x480.jpg \
    --iterations 200
```

Record: mean, p50, p95, p99 inference latency. Run with `sar -b 1` in
parallel to measure DDR bus utilisation during NPU inference — this
quantifies memory contention with the CPU-side ExG pipeline.

Targets to confirm before proceeding to M5:

| Metric | Target |
|---|---|
| YOLOX-Nano mean inference | < 80 ms (≥ 12 Hz) |
| CPU pipeline latency during NPU inference | < 25 ms |
| DDR utilisation peak | < 70% |

If CPU pipeline latency degrades significantly during NPU inference,
introduce a frame-skip strategy: NPU inference runs every N-th frame,
`crop_row_node` runs every frame using the most recent available detections.

---

### M5 · Composable node container integration (Week 5–6)

Formalise the shared-topic, intra-process architecture from section 1 into
the production launch file. Verify with:

```bash
ros2 component list
# All three nodes should appear under /vision_container

ros2 topic hz /caatinga_vision/row_nav/image_raw
# Single publisher at 30 Hz
```

Confirm that intra-process communication is active by checking ROS 2 logs
for `"Intra-process communication enabled"` on startup for each subscribing
composable node.

---

## 4. Medium-Term — Replace ExG with NPU Segmentation
### (Months 2–4)

This is the highest-impact change and the most work. ExG with weed masking
(M2) is the bridge — it must be stable in production before this work
begins.

---

### M6 · Training data collection (Months 1–3, parallel workstream)

This is the long-lead item that gates everything in section 4.
Begin immediately in parallel with M0–M5.

**Target: 1,200 pixel-wise labelled images, three classes:**
`{lettuce_row, weed, soil/background}`

Capture requirements for UK outdoor lettuce:

| Condition | Target images |
|---|---|
| Overcast (typical UK) | 400 |
| Bright sun / partial cloud | 200 |
| Early morning / low-angle light | 150 |
| Post-rain / wet soil with mud splash | 150 |
| Emergence stage (small, sparse plants) | 150 |
| Canopy closure stage | 150 |

**Semi-automatic labelling strategy** — use existing models as labelling
tools rather than labelling from scratch:

1. Run YOLOX-Nano detections → weak weed labels
2. Run normalised ExG + Otsu → weak crop/soil labels
3. Manually correct disagreements, edge cases, plant boundaries,
   mud-splashed leaves

This reduces manual labelling effort by approximately 60–70% compared to
labelling from scratch. Tools: CVAT (self-hosted), Label Studio, or
Segments.ai.

**Useful pretraining datasets** (European, directly applicable to UK):
- CropAndWeed Dataset — Steininger et al., 2023
- PhenoBench — Weyler et al., 2023
- WeedMap — Ramirez et al., 2020

**India adaptation:** A separate 400-image set captured locally is required
before Indian field deployment. Weed flora, soil colour, illumination
conditions, and crop appearance differ enough from UK that a UK-trained
model will degrade significantly without domain adaptation. Do not skip
this step.

---

### M7 · Segmentation model selection and training (Month 2–4)

**Target architecture: MobileNetV2 + LR-ASPP at 320×240 input.**

| Architecture | Input | Est. Z2 NPU latency | Verdict |
|---|---|---|---|
| MobileNetV2 + LR-ASPP | 320×240 | ~15–25 ms | ✅ Start here |
| STDC-Seg (STDC1) | 320×240 | ~10–20 ms | ✅ Evaluate second |
| ERFNet | 320×240 | ~30–50 ms | ⚠️ Marginal budget |
| SegNet | any | > 60 ms | ❌ Too heavy |

Do not use 640×480 input for the first deployed version. 320×240 reduces
NPU compute and DDR transfer by 4×. The downstream `detect_crop_rows`
polyfit is stable at half resolution — lettuce rows are regular enough that
sub-pixel row-centre accuracy is not required.

All latency estimates are from comparable 2–4 TOPS embedded NPU benchmarks.
Validate on T527 before committing.

**Training procedure:**

1. Pretrain on CropAndWeed + PhenoBench (three classes remapped to
   `lettuce_row / weed / background`)
2. Fine-tune on UK field dataset (M6)
3. Quantise to INT8 using Zhouyi Compass Optimizer (`AIPUBuilder`) with UK
   field images as calibration data — do not use ImageNet calibration
4. Acceptance criterion: INT8 mIoU must be within 3 percentage points of
   float model mIoU before deployment

---

### M8 · Integration into `crop_row_node` (Month 3–4)

The segmentation output replaces exactly two function calls. Everything
downstream is unchanged:

```python
# BEFORE
mask    = compute_exg_mask(bgr)
centers = get_plant_centers(mask, self.min_area)

# AFTER — seg_mask delivered via /seg_mask topic (mono8)
# published by weed_detect_node after NPU inference
seg_mask  = self._latest_seg_mask          # 0=soil, 1=crop, 2=weed
crop_mask = (seg_mask == 1).astype(np.uint8) * 255
weed_mask = (seg_mask == 2).astype(np.uint8) * 255
centers   = get_plant_centers(crop_mask, self.min_area)
```

The dense weed mask replaces the YOLOX bounding box exclusion from M2 —
pixel-accurate, no confidence thresholding needed for the navigation path.

**NPU time-sharing strategy** (Zhouyi Z2 runs models sequentially):

- **Segmentation:** every frame at 30 Hz (~15–25 ms), primary navigation input
- **YOLOX-Nano:** every 3rd frame at 10 Hz, used for precise weed
  localisation to drive treatment actuation (sprayer trigger / mechanical
  weeder position), not for navigation

At 0.15 m/s forward speed, 10 Hz YOLOX gives a weed position update every
15 mm of travel — sufficient for actuation timing on a robot at this speed.

---

## 5. Long-Term — Semantic Field Navigation
### (Months 6–12)

---

### M9 · Row-end and headland detection (Month 6)

The current node has no concept of reaching the end of a row. Add a
`headland` class to the segmentation model trained in M7. Publish:

```
/aoc/conditions/row_end   (std_msgs/Bool)
```

The Limbic System uses this to trigger headland turning. This eliminates
the need for a separate sensor or waypoint-based row-end detection.

---

### M10 · Row identity and cross-row localisation (Month 7–8)

The polyfit gives row-relative position but no global field map. After
segmentation is stable, add a row-counter derived from frame-to-frame
tracking of the lateral phase of detected row positions. This allows the
Limbic System to maintain which row the robot is in, enabling:

- Skip-row weeding patterns
- Per-row treatment records
- Return-to-row after interruption or e-stop

No additional sensors required — derived entirely from the existing
segmentation output.

---

### M11 · Weed pressure maps (Month 8–9)

Accumulate weed detections across the field geo-referenced by wheel
odometry or RTK-GPS. Publish as:

```
/aoc/maps/weed_pressure   (nav_msgs/OccupancyGrid)
```

This produces per-row, per-bed weed density estimates — the primary
agronomic data output of the system. In UK commercial lettuce production
this also supports the field records required under Red Tractor and LEAF
Marque standards, even for mechanical weeding operations.

---

### M12 · Multi-task unified network (Month 9–12)

Replace the two-model time-sharing arrangement with a single network:

```
Shared MobileNetV2 encoder
  │
  ├─► Segmentation head   (lettuce / weed / soil / headland)
  └─► Detection head      (weed bounding boxes for actuation)
```

Single NPU inference per frame. Shared feature extraction removes
approximately 60% of redundant compute. Both outputs are temporally aligned
with no synchroniser required.

This is a 3–4 month model development project and should not be started
before the single-task segmentation model (M7) has been validated and
stable in production for at least one growing season.

---

## Dependency and Risk Register

| Item | Severity | Status | Mitigation |
|---|---|---|---|
| ExG corrupted by weed pressure | **Critical** | Active now | M2 immediately; M8 resolves permanently |
| YOLOX false positives corrupting row centre | **High** | Active when M2 deployed | Confidence threshold ≥ 0.55; validate M3 first |
| T527 NPU latency unverified | **High** | Blocks M5+ | Benchmark (M4) before any Hz commitments |
| UK field labelling dataset — long lead time | **High** | Begin immediately | Semi-automatic labelling; parallel workstream from day 1 |
| India domain shift vs UK-trained model | **High** | Future deployment | 400-image local adaptation set; re-quantise with local calibration |
| DDR bus contention CPU + NPU concurrent | Medium | Measure at M4 | Frame-skip if needed; time-sharing in M8 |
| Mud splash degrading ExG in wet UK conditions | Medium | Active | M1 normalisation reduces impact; M8 resolves permanently |
| Composable node intra-process not activating | Low | Verify at M5 | Check ROS 2 logs on startup; fall back to standard pub/sub if needed |

---

## Milestone Summary

| # | Milestone | When | Output | Prerequisite |
|---|---|---|---|---|
| **M0** | Baseline profiling | Week 1 | Measured latency budget on T527 | — |
| **M1** | Normalised ExG | Week 1–2 | Illumination-robust perception, one function | M0 |
| **M2** | Weed exclusion mask | Week 2–3 | YOLOX boxes suppress ExG false positives | — |
| **M2.1** | Parameterise dilation kernel | Week 2–3 | `crop_row_params.yaml` tuneable per crop stage | — |
| **M3** | YOLOX UK weed validation | Week 3–5 | Per-species P/R, confidence threshold set | M2 production trust |
| **M4** | NPU latency benchmark | Week 4–6 | Confirmed Hz budget, DDR contention profile | M5, M7 |
| **M5** | Composable node container | Week 5–6 | Zero-copy image sharing, production launch | M4 |
| **M6** | UK field dataset *(parallel)* | Month 1–3 | 1,200 labelled images, three classes | M7 |
| **M7** | Segmentation model v1 | Month 2–4 | MobileNetV2+LR-ASPP, INT8, on NPU | M6, M4 |
| **M8** | Unified segmentation pipeline | Month 3–4 | ExG removed, seg 30 Hz, YOLOX 10 Hz | M7 stable |
| **M9** | Row-end / headland detection | Month 6 | `/aoc/conditions/row_end`, Limbic turning | M8 |
| **M10** | Row identity tracking | Month 7–8 | Row counter, return-to-row | M8 |
| **M11** | Weed pressure maps | Month 8–9 | `OccupancyGrid`, per-row agronomic records | M10 |
| **M12** | Multi-task unified network | Month 9–12 | Single NPU inference, seg + detection | M8 one season stable |
