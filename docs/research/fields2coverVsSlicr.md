# slic3r_coverage_planner vs Fields2Cover

## What they are

**slic3r_coverage_planner** is a ROS package that repurposes the slicing engine from the Slic3r 3D printer slicer as a lawn-mowing path planner. It's the coverage planner used in the OpenMower project, specifically used to plan the mowing path. It generates infill-style parallel lines and perimeter passes, directly borrowing 3D printing concepts.

**Fields2Cover** is a dedicated, academically-developed coverage path planning library. It provides a framework for planning coverage paths, developing novel techniques, and benchmarking state-of-the-art algorithms, with a modular and extensible architecture that supports various vehicles.

---

## slic3r_coverage_planner

### Pros

- **Zero algorithm work required.** The Slic3r slicing engine is battle-hardened from years of 3D printing use; perimeter passes and infill generation work out of the box.
- **Already integrated into OpenMower.** It's natively used in the OpenMower ROS stack alongside teb_local_planner and xesc_ros. If you're on that platform, it's the path of least resistance.
- **Handles islands and perimeters naturally.** 3D slicer infill logic inherently understands polygon interiors, holes, and perimeter offsets — concepts that map directly to mowing.
- **Simpler operational model.** The workflow is straightforward: drive the mower around the perimeter with a handheld controller, mark obstacles, and set it going.

### Cons

- **Not designed for robotics.** The core logic is borrowed from 3D printing; it has no concept of vehicle kinematics, turn radius, or headland management.
- **No route optimization.** There's no ability to order swaths optimally or minimize non-productive travel between passes.
- **AGPL-3.0 license.** This is a copyleft license that can create complications for commercial or proprietary use.
- **Small, niche community.** Only 44 stars and a handful of contributors — limited ongoing development outside OpenMower's needs.
- **ROS1-centric origin.** The rs64-ctrl ROS2 Jazzy fork is a community port, not an official release, meaning less guarantee of long-term maintenance.
- **No academic benchmarks or objective functions.** You can't compare or tune the planner's efficiency in any principled way.

---

## Fields2Cover

### Pros

- **Purpose-built for coverage planning with proper algorithms.** Its core modules are a headland generator, swath generator, route planner, and path planner — all the right abstractions for a real mowing/farming robot.
- **Non-convex fields and obstacles (v2.0).** Version 2.0 brings support for non-convex fields and fields with obstacles, using Trapezoidal and Boustrophedon decomposition algorithms to split any concave field into several convex sub-fields.
- **Route optimization with OR-Tools.** A route optimizer using OR-Tools can be used to order the swaths, instead of just using a known pattern, with support for a defined start and end point.
- **Multiple turn types.** Fields2Cover supports straight curves, Dubins' curves, and Reeds-Shepp curves for headland turns — critical for real kinematically constrained vehicles.
- **Python + C++ interface.** Implemented in C++17 with a Python interface using SWIG.
- **Native ROS1 and ROS2 support.** An interface with ROS1 and ROS2 is provided as an add-on, with RViz support to visualize results.
- **Academically validated.** The library demonstrates 8 state-of-the-art methods and 7 objective functions, with a peer-reviewed IEEE RA-L paper backing it.
- **Permissive BSD-3 license.** Safe for commercial or proprietary projects.
- **Wageningen University backing.** Well-maintained, with a clear governance structure and migration guides between versions.

### Cons

- **More complex to integrate.** The modular architecture requires understanding headland generation, swath generation, route planning, and path planning as separate steps — more setup overhead vs. a single-call planner.
- **Heavier dependency footprint.** Requires OR-Tools, GDAL, and other dependencies; compilation is non-trivial compared to simply cloning into a ROS workspace.
- **Primarily designed for agricultural vehicles.** Tuning it for a small domestic lawn mower may require more work than for a tractor-scale machine.
- **Offline planning only.** Development is focused on offline planning of agricultural vehicles — not reactive or online replanning.

---

## Summary

| Dimension | slic3r_coverage_planner | Fields2Cover |
|---|---|---|
| Integration effort | Low (for OpenMower) | Medium–High |
| Algorithm quality | Basic | State-of-the-art |
| Non-convex support | Limited | ✅ Full (v2.0) |
| Route optimization | ❌ | ✅ OR-Tools |
| Kinematics-aware turns | ❌ | ✅ Dubins/RS |
| ROS2 support | Community port | Official |
| License | AGPL-3.0 | BSD-3 |
| Community/docs | Small | Strong |

If you're building on top of **OpenMower and want the quickest path to a working system**, slic3r_coverage_planner is already there. If you're building a **more capable, general-purpose, or commercial** autonomous mowing/agricultural system, Fields2Cover is the clearly superior foundation.
