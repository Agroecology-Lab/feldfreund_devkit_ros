# Coverage Planner Feature Comparison
## slic3r_coverage_planner (current OpenMower) vs Fields2Cover

> Reference: current slic3r behaviour as described by the OpenMower community.

---

## Feature-by-feature breakdown

### 1. Outline covered by rounds, inner to outer

| | slic3r | Fields2Cover |
|---|---|---|
| **Status** | ✅ Works well | ⚠️ Partial |
| **Notes** | Generates multiple perimeter passes as a native 3D-printing concept — inward offset rings are a first-class feature. | The headland generator creates a single headland band of configurable width (expressed as multiples of robot width). There is currently **only one headland generator** (`ConstHL`), and it does not natively produce multiple inward-offset perimeter passes as separate driving tracks the way slic3r does. You would need to call it repeatedly with shrinking widths to replicate this. |

---

### 2. Rounds around obstacles

| | slic3r | Fields2Cover |
|---|---|---|
| **Status** | ✅ Works well | ⚠️ Requires workaround |
| **Notes** | Slic3r treats obstacles as island polygons and naturally generates perimeter passes around them, exactly as it does for the outer border. | F2C v2.0 supports obstacles and non-convex fields via decomposition, but it does not generate dedicated perimeter passes *around obstacles* the way slic3r does. Obstacle areas are excluded from swaths; navigating around them in perimeter-pass style would need to be implemented separately. |

---

### 3. Rounds near outline and obstacle merged into one track

| | slic3r | Fields2Cover |
|---|---|---|
| **Status** | ✅ Works well | ❌ Not supported |
| **Notes** | When an obstacle is close enough to the outer boundary, slic3r intelligently merges the perimeter passes into a single combined track, avoiding a tiny gap that would be impractical to mow. | Fields2Cover has no equivalent merging logic. Headland and obstacle exclusion zones are computed independently. A narrow strip between an obstacle's exclusion zone and the field's headland zone would either be included in the fill swaths (possibly poorly) or dropped entirely. This would need custom post-processing. |

---

### 4. Smooth transitions between rounds

| | slic3r | Fields2Cover |
|---|---|---|
| **Status** | ✅ Works well | ✅ Supported, and better |
| **Notes** | Slic3r generates smooth spiral-like transitions between perimeter rings. | F2C's path planner supports Dubins curves, Dubins with continuous curvature (CC), Reeds-Shepp curves, and Reeds-Shepp CC. The CC variants explicitly smooth out instantaneous curvature changes, making the result more physically accurate for a real vehicle with a minimum turning radius. This is *more* capable than slic3r's transitions, but applies to swath-to-swath turns rather than ring-to-ring transitions specifically. |

---

### 5. Fill using parallel lines

| | slic3r | Fields2Cover |
|---|---|---|
| **Status** | ✅ Works well | ✅ Fully supported |
| **Notes** | Standard infill pattern from 3D printing. | F2C's swath generator produces parallel line fills natively. The `BruteForce` generator finds the optimal angle to minimise number of swaths. Multiple route ordering patterns are available: Boustrophedon (simple sequential), Snake (skip-one-return, reduces sharp turns), Spiral, and a fully optimised order via OR-Tools. This is richer than slic3r's fill ordering. |

---

### 6. Multiple fills for complex/concave shapes

| | slic3r | Fields2Cover |
|---|---|---|
| **Status** | ✅ Works well | ✅ Supported, explicit step required |
| **Notes** | Slic3r automatically detects when a single fill region doesn't work and splits into multiple regions — this is implicit and requires no configuration. | F2C v2.0 achieves this via the **cell decomposition** step (Trapezoidal or Boustrophedon decomposition). For an L-shaped field, decomposing first then applying swath generation can reduce swath count dramatically (e.g. from 84 to 26 in the documented example). However, this is an *explicit* pipeline step — the user must decide when to decompose and configure it. It is not automatic. The decomposition also requires a specific two-pass headland workflow to keep the route planner connected across decomposed cells. |

---

### 7. Consistent fill angle across all regions, easily configurable

| | slic3r | Fields2Cover |
|---|---|---|
| **Status** | ✅ Works well | ✅ Fully supported |
| **Notes** | One angle parameter applies to all fill regions. | F2C passes the sweep angle directly to the swath generator as a parameter. When using decomposition across multiple cells, all cells can be given the same angle. The `BruteForce` generator can also *find* the optimal angle automatically if you prefer that over a fixed value. |

---

### 8. Half-circle turns between fill lines

| | slic3r | Fields2Ford |
|---|---|---|
| **Status** | ✅ Works well | ✅ Supported, and more flexible |
| **Notes** | Fixed half-circle U-turns between parallel swaths. | F2C supports four turn types: straight, Dubins, Dubins-CC, and Reeds-Shepp(-CC). For a non-holonomic mower with a minimum turning radius, Dubins or Dubins-CC produces the shortest valid turn — which may be a half-circle or tighter, depending on the robot parameters. This is physically more correct than a fixed half-circle, but requires setting `robot.setMinTurningRadius()` correctly. |

---

### 9. Inter-path navigation done at runtime via costmap (not pre-planned)

| | slic3r | Fields2Cover |
|---|---|---|
| **Status** | ✅ Deliberate design choice | ✅ Pre-planned, but nav stack still usable |
| **Notes** | Transitions between outlines, obstacle rounds, and fill regions are not part of the planned path — the nav stack handles them dynamically using a costmap. This keeps the planner simple and lets obstacle avoidance handle surprises. | F2C's route planner (using OR-Tools or known patterns) generates the full ordered sequence of swaths *and* the headland traversal paths connecting them. The complete path including transitions is output as a single `F2CPath`. This is more deterministic and efficient, but means the pre-planned transitions could conflict with unexpected runtime obstacles — you still need a local planner (e.g. TEB) on top for safety. |

---

### 10. Paths ordered in a sensible way

| | slic3r | Fields2Cover |
|---|---|---|
| **Status** | ✅ Works reasonably | ✅ More principled, and optimisable |
| **Notes** | Slic3r uses printing-derived heuristics to order paths in a practical way, but there is no formal optimisation. | F2C offers multiple explicit strategies: Boustrophedon (simple sequential), Snake (fewer sharp turns), Spiral (useful for capacity-limited machines), and a full OR-Tools optimiser that finds the globally shortest route. A defined start/end point can also be enforced. This is strictly more capable, at the cost of needing to choose a strategy. |

---

### 11. Input as polygon with holes; border = mower centre line

| | slic3r | Fields2Cover |
|---|---|---|
| **Status** | ✅ Works well | ✅ Fully supported |
| **Notes** | The input polygon represents the area reachable by the mower's centre, with holes for exclusion zones. Slic3r handles this natively. | F2C accepts `F2CCells` (polygons with holes) as input. The headland generator shrinks the field boundary inward by the desired margin, so the border of the input polygon should be the mower's centre-line boundary — exactly the same convention. GeoJSON input is also supported for real GPS-referenced fields. |

---

## Summary matrix

| Feature | slic3r (OpenMower) | Fields2Cover |
|---|---|---|
| Outline perimeter passes (multiple rounds) | ✅ Native | ⚠️ Manual multi-call workaround |
| Perimeter passes around obstacles | ✅ Native | ⚠️ Not built-in |
| Merge nearby outline/obstacle rounds | ✅ Automatic | ❌ Not supported |
| Smooth ring-to-ring transitions | ✅ Spiral-style | ✅ Kinematically correct curves |
| Parallel line fill | ✅ | ✅ |
| Multiple fills for concave shapes | ✅ Automatic | ✅ Explicit decomposition step |
| Consistent configurable fill angle | ✅ | ✅ (plus auto-optimise) |
| Half-circle / U-turn between swaths | ✅ Fixed half-circle | ✅ Kinematically optimal turns |
| Inter-path navigation pre-planned | ❌ Runtime costmap | ✅ Pre-planned (+ local planner still needed) |
| Sensible path ordering | ✅ Heuristic | ✅ Multiple strategies + OR-Tools |
| Polygon-with-holes input, centre-line border | ✅ | ✅ |

---

## Bottom line

Fields2Cover is **more powerful** on fill ordering, turn geometry, and route optimisation. However, slic3r has two behaviours that Fields2Cover **does not replicate cleanly out of the box**:

1. **Perimeter passes around obstacles** — F2C excludes obstacle areas but doesn't orbit them with offset rings.
2. **Automatic merging of obstacle rounds with outline rounds** — this contextual geometry simplification is unique to slic3r's approach and would require custom logic in F2C.

For a migration to F2C, these two features would likely need bespoke implementation on top of the F2C primitives.
