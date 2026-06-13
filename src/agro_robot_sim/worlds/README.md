# worlds/

The `.world` / `.sdf` files committed here are **source assets** (the Caatinga
farm scenes: `minha_fazenda`, `empty`, …) — keep them tracked.

`maize.world`, used by the sim topo-nav loop, is **not** here: it is a generated
artefact produced at runtime by the forest3d pipeline
(`topo_to_forest3d.py` → `forest3d.yaml` → `.world`) and written into the
install space, e.g. `install/agro_robot_sim/share/agro_robot_sim/worlds/maize.world`.
It is regenerated on each sim boot / "Rebuild World", so it is intentionally not
committed.
