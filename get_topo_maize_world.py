#!/usr/bin/env python3
"""
get_topo_maize_world.py — reverse of get_maize_topo.py.

Reads a saved topological map (the R*_IN / R*_OUT nodes authored in the
devkit_ui cockpit) and emits a Gazebo (Harmonic / Jazzy) SDF world that
places virtual_maize_field maize models in the INTER-ROW GAPS, so the
robot drives the clear lanes the topo rows describe.

Pipeline (inverts the old gt_map.csv -> get_maize_topo.py direction):

    saved topo map  ->  this script  ->  maize.world  ->  Gazebo

Each topo "row" is the pair (R{rid}_IN, R{rid}_OUT) sharing a row_id. The
robot follows the line between IN and OUT. Plants are studded along the
midlines BETWEEN adjacent rows, never on the driving lanes themselves.

Usage:
    python3 get_topo_maize_world.py \
        --topo /workspace/maps/maize_map \
        --out  /workspace/install/agro_robot_sim/share/agro_robot_sim/worlds/maize.world \
        --plant-spacing 0.15

Model URIs use model://maize_01 / model://maize_02, which resolve via
GZ_SIM_RESOURCE_PATH (already set to the virtual_maize_field models dir in
both manage.py and the UI _SIM_ENV).
"""

import argparse
import json
import math
import random
import sys
from pathlib import Path
from datetime import datetime

# YAML is optional: the LCAS TMAP2 file is YAML, but the UI also persists
# JSON. Try YAML first, fall back to JSON so the script runs even where
# pyyaml is absent.
try:
    import yaml  # type: ignore
    _HAVE_YAML = True
except Exception:
    _HAVE_YAML = False


# ── topo parsing ──────────────────────────────────────────────────────────────

def load_topo(path):
    """Return {node_name: {'x', 'y', 'row_id', 'row_role'}} for every node.

    Accepts either the LCAS TMAP2 YAML or the UI's JSON dump. Both share the
    same shape: doc['nodes'] is a list of entries, each with entry['node']
    holding name / pose.position and entry-or-node 'properties'.
    """
    text = Path(path).read_text()
    doc = None
    if _HAVE_YAML:
        try:
            doc = yaml.safe_load(text)
        except Exception:
            doc = None
    if doc is None:
        try:
            doc = json.loads(text)
        except Exception:
            sys.exit(f"ERROR: cannot parse {path} as YAML or JSON")

    entries = doc.get('nodes', [])
    if not entries:
        sys.exit(f"ERROR: no nodes in {path}")

    nodes = {}
    for entry in entries:
        n = entry.get('node', entry)
        name = n.get('name')
        pos = n.get('pose', {}).get('position', {})
        if name is None or 'x' not in pos or 'y' not in pos:
            continue
        # row_id / row_role may sit in meta or in node.properties
        props = n.get('properties', {})
        meta = entry.get('meta', {})
        row_id = props.get('row_id', meta.get('row_id'))
        row_role = props.get('row_role', meta.get('row_role'))
        nodes[name] = {
            'x': float(pos['x']),
            'y': float(pos['y']),
            'row_id': row_id,
            'row_role': (row_role or '').lower() if row_role else None,
        }
    if not nodes:
        sys.exit(f"ERROR: nodes present but none had a usable pose in {path}")
    return nodes


def extract_rows(nodes):
    """Group IN/OUT endpoints by row_id into row centre-segments.

    Returns a list of rows sorted across the field, each:
        {'rid', 'a': (x,y), 'b': (x,y)}   # a=IN endpoint, b=OUT endpoint
    Rows without a clean IN+OUT pair are skipped with a warning.
    """
    by_rid = {}
    for name, nd in nodes.items():
        rid = nd['row_id']
        if rid is None:
            continue
        by_rid.setdefault(rid, {})[nd['row_role'] or name] = nd

    rows = []
    for rid, ends in by_rid.items():
        in_nd = ends.get('entry')
        out_nd = ends.get('exit')
        # Fall back to name-suffix matching if role wasn't recorded.
        if in_nd is None or out_nd is None:
            ins = [v for k, v in ends.items() if str(k).upper().endswith('IN')]
            outs = [v for k, v in ends.items() if str(k).upper().endswith('OUT')]
            in_nd = in_nd or (ins[0] if ins else None)
            out_nd = out_nd or (outs[0] if outs else None)
        if in_nd is None or out_nd is None:
            print(f"  skip row {rid}: need both IN and OUT (got {list(ends)})")
            continue
        rows.append({'rid': rid,
                     'a': (in_nd['x'], in_nd['y']),
                     'b': (out_nd['x'], out_nd['y'])})

    if len(rows) < 2:
        sys.exit(f"ERROR: need >=2 complete rows to place inter-row plants; "
                 f"found {len(rows)}")

    # Sort rows across the field by their cross-axis position. Determine which
    # axis is "across" by comparing total spread of row-centres in x vs y.
    cxs = [(r['a'][0] + r['b'][0]) / 2 for r in rows]
    cys = [(r['a'][1] + r['b'][1]) / 2 for r in rows]
    x_spread = max(cxs) - min(cxs)
    y_spread = max(cys) - min(cys)
    cross = 'x' if x_spread >= y_spread else 'y'
    rows.sort(key=lambda r: (r['a'][0] + r['b'][0]) / 2 if cross == 'x'
              else (r['a'][1] + r['b'][1]) / 2)
    return rows, cross


# ── geometry ──────────────────────────────────────────────────────────────────

def midline(rowA, rowB):
    """Return ((x0,y0),(x1,y1)) — the line halfway between two row segments.

    Uses IN endpoints for the start, OUT endpoints for the end, so the
    midline runs the same direction the robot travels.
    """
    ax0, ay0 = rowA['a']; ax1, ay1 = rowA['b']
    bx0, by0 = rowB['a']; bx1, by1 = rowB['b']
    return ((((ax0 + bx0) / 2), ((ay0 + by0) / 2)),
            (((ax1 + bx1) / 2), ((ay1 + by1) / 2)))


def stud_line(p0, p1, spacing, placement_err, rng):
    """Yield (x, y, yaw) plant poses along p0->p1 at ~spacing metres."""
    x0, y0 = p0; x1, y1 = p1
    dx, dy = x1 - x0, y1 - y0
    length = math.hypot(dx, dy)
    if length < 1e-6 or spacing <= 0:
        return
    yaw = math.atan2(dy, dx)
    nx, ny = -dy / length, dx / length  # unit normal, for lateral jitter
    n = max(1, int(round(length / spacing)))
    for i in range(n + 1):
        t = i / n
        x = x0 + dx * t
        y = y0 + dy * t
        if placement_err > 0:
            e = rng.uniform(-placement_err, placement_err)
            x += nx * e
            y += ny * e
        yield x, y, yaw


# ── SDF emission ──────────────────────────────────────────────────────────────

def sdf_header(name):
    return f"""<?xml version="1.0" ?>
<!-- Auto-generated by get_topo_maize_world.py {datetime.now().isoformat(timespec='seconds')} -->
<!-- Plants placed in inter-row gaps derived from saved topo R*_IN/OUT nodes. -->
<sdf version="1.7">
  <world name="{name}">
    <physics name="default" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.2 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <material>
            <ambient>0.3 0.25 0.18 1</ambient>
            <diffuse>0.4 0.32 0.22 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""


def sdf_include(inst_name, model_uri, x, y, yaw, scale):
    scale_line = ""
    if abs(scale - 1.0) > 1e-6:
        scale_line = f"      <scale>{scale:.3f} {scale:.3f} {scale:.3f}</scale>\n"
    return f"""    <include>
      <name>{inst_name}</name>
      <uri>model://{model_uri}</uri>
      <pose>{x:.3f} {y:.3f} 0 0 0 {yaw:.3f}</pose>
{scale_line}    </include>
"""


def sdf_footer():
    return "  </world>\n</sdf>\n"


def generate(topo_path, out_path, world_name, crop_types, spacing,
             placement_err, seed, skip_prob, plant_scale):
    rng = random.Random(None if seed < 0 else seed)
    nodes = load_topo(topo_path)
    rows, cross = extract_rows(nodes)
    print(f"Loaded {len(rows)} rows (cross-axis={cross}); "
          f"placing maize in {len(rows) - 1} inter-row gaps "
          f"(scale={plant_scale})")

    out = sdf_header(world_name)
    plant_idx = 0
    for k in range(len(rows) - 1):
        p0, p1 = midline(rows[k], rows[k + 1])
        gap_n = 0
        for x, y, yaw in stud_line(p0, p1, spacing, placement_err, rng):
            if skip_prob > 0 and rng.random() < skip_prob:
                continue  # simulate the occasional missing plant
            model = crop_types[plant_idx % len(crop_types)]
            out += sdf_include(f"plant_{plant_idx:04d}", model, x, y, yaw,
                               plant_scale)
            plant_idx += 1
            gap_n += 1
        print(f"  gap {rows[k]['rid']}->{rows[k+1]['rid']}: {gap_n} plants")
    out += sdf_footer()

    Path(out_path).parent.mkdir(parents=True, exist_ok=True)
    Path(out_path).write_text(out)
    print(f"Wrote {plant_idx} plants -> {out_path}")


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--topo', default='/workspace/maps/maize_map',
                    help='Saved topo map (LCAS TMAP2 YAML or UI JSON).')
    ap.add_argument('--out',
                    default='/workspace/install/agro_robot_sim/share/'
                            'agro_robot_sim/worlds/maize.world',
                    help='Output SDF world path.')
    ap.add_argument('--name', default='maize_field',
                    help='SDF <world> name.')
    ap.add_argument('--crop-types', nargs='+', default=['maize_01', 'maize_02'],
                    help='virtual_maize_field model names to alternate.')
    ap.add_argument('--plant-spacing', type=float, default=0.15,
                    help='Intra-row plant spacing along the gap midline (m).')
    ap.add_argument('--placement-error', type=float, default=0.02,
                    help='Max lateral jitter per plant (m).')
    ap.add_argument('--skip-prob', type=float, default=0.0,
                    help='Per-plant probability of a gap (missing plant).')
    ap.add_argument('--plant-scale', type=float, default=0.33,
                    help='Uniform scale applied to each maize model '
                         '(0.33 = one-third size / much shorter).')
    ap.add_argument('--seed', type=int, default=-1,
                    help='RNG seed (-1 = nondeterministic).')
    args = ap.parse_args()

    generate(args.topo, args.out, args.name, args.crop_types,
             args.plant_spacing, args.placement_error, args.seed,
             args.skip_prob, args.plant_scale)
