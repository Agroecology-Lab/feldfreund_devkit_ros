#!/usr/bin/env python3
"""
topo_to_forest3d.py — bridge between Sowbot topo maps and Forest3D.

Reads a saved topological map (the R*_IN / R*_OUT nodes authored in the
devkit_ui cockpit) and generates a forest3d.yaml config + optional world.

Pipeline:
    saved topo map  ->  this script  ->  forest3d.yaml
                                       ->  forest3d generate  ->  .world

Each topo "row" is the pair (R{rid}_IN, R{rid}_OUT) sharing a row_id. The
robot drives the line between IN and OUT (the furrow). Forest3D's crop_rows
generates raised beds (crop rows) with furrows between — the driving lanes.
"""

import argparse
import json
import math
import re
import subprocess
import sys
from pathlib import Path

try:
    import yaml
except ImportError:
    yaml = None

# Weeds are placed per crop plant. A weedy field carries several weeds for
# every crop, so --weed-density 100 (i.e. 100%) maps to this many weeds per
# plant rather than one. Raise for a denser field, lower for a cleaner one.
WEEDS_PER_CROP = 3

# Forest3D's Forest3DConfig pydantic schema hard caps (crop_rows.num_rows,
# density.crop) — see forest3d/config/loader.py's ValidationError on any
# field that exceeds these. Sized for a small demo field; a real F2C-planned
# full-field coverage plan (hundreds of rows) blows straight through them.
# This is a VISUALIZATION-ONLY limit: it only affects how many rows get
# rendered as Gazebo terrain/crop models via this script. It never touches
# the actual navigation topo map (maps/maize_map), which keeps its full,
# real row count regardless — see the clipping note in main() below.
FOREST3D_MAX_ROWS = 200
FOREST3D_MAX_DENSITY = 10000

# Deliberate target for how many rows to actually render in Gazebo, well
# below the hard caps above — a real multi-hundred-row field is heavy to
# simulate even at row counts the schema would technically accept, so this
# keeps the visual scene small and fast regardless of field size.
VISUAL_TARGET_ROWS = 8

# Fallback GPS origin when a topo map carries no gps_lat/gps_lon tags at all
# (e.g. the synthetic virtual_maize_field map, which has no real-world
# georeference). Without SOME origin, gz-sim-navsat-system falls back to its
# own hardcoded (0, 0) — see patch_world_spherical_coordinates' docstring —
# and every /gnss/fix is rejected as out of range. This must match the
# fake-GPS shim in ui_node.py (_FAKE_GPS_LAT/LON) and the leaflet defaults
# there, and test_contour_planning.py's --anchor-lat/lon default — ui_node.py
# explicitly warns that a mismatch above ~53m between its shim and the map's
# back-solved origin trips FusionCore's outlier gate and silently rejects
# every subsequent real GPS fix for the rest of the run. Do not change this
# without updating those in lockstep.
DEFAULT_FIELD_LAT = 48.0046000
DEFAULT_FIELD_LON = 3.6644000

def load_topo(path):
    text = Path(path).read_text()
    doc = None
    if yaml:
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
        props = n.get('properties', {})
        meta = entry.get('meta', {})
        row_id = props.get('row_id', meta.get('row_id'))
        row_role = props.get('row_role', meta.get('row_role'))
        gps_lat = props.get('gps_lat', meta.get('gps_lat'))
        gps_lon = props.get('gps_lon', meta.get('gps_lon'))
        nodes[name] = {
            'name': name,
            'x': float(pos['x']),
            'y': float(pos['y']),
            # Outgoing edge targets, in file order. extract_rows() follows the
            # entry -> waypoint -> exit chain through these to recover the
            # real (possibly curved) row geometry.
            'edges': [e.get('node') for e in (n.get('edges') or [])
                      if isinstance(e, dict) and e.get('node')],
            'row_id': row_id,
            'row_role': (row_role or '').lower() if row_role else None,
            'gps_lat': float(gps_lat) if gps_lat else None,
            'gps_lon': float(gps_lon) if gps_lon else None,
        }
    if not nodes:
        sys.exit(f"ERROR: nodes present but none had a usable pose in {path}")
    return nodes


def find_spawn_node(nodes, rows):
    """Pick the topo node the robot should spawn at, in topo-frame coords.

    Uses the FIRST node the user placed (skip any literal 'HOME', which the
    bootstrapped get_maize_topo.py map pins to (0,0) regardless of the real
    field position). Falls back to the first row's entry node.
    """
    for name, nd in nodes.items():
        if name == 'HOME':
            continue
        return nd['x'], nd['y']
    if rows:
        return rows[0]['a']
    return 0.0, 0.0


def _order_row_chain(members, in_nd, out_nd):
    """Return the row's nodes ordered entry -> waypoints -> exit.

    Prefers the authored edge chain (entry -> W1 -> ... -> exit, as written by
    the UI's contour/F2C row saver). If that chain is broken or skips any
    waypoint, falls back to the waypoints' trailing number (R2_W3 -> 3), then
    to their projection onto the entry->exit chord.
    """
    by_name = {nd['name']: nd for nd in members}
    waypoints = [nd for nd in members
                 if nd is not in_nd and nd is not out_nd]

    chain, seen, cur = [in_nd], {in_nd['name']}, in_nd
    while cur is not out_nd:
        nxt = next((by_name[t] for t in cur['edges']
                    if t in by_name and t not in seen), None)
        if nxt is None:
            break
        chain.append(nxt)
        seen.add(nxt['name'])
        cur = nxt
    if cur is out_nd and len(chain) == len(waypoints) + 2:
        return chain

    def trailing_int(nd):
        m = re.search(r'(\d+)$', nd['name'])
        return int(m.group(1)) if m else None

    if waypoints and all(trailing_int(w) is not None for w in waypoints):
        waypoints.sort(key=trailing_int)
    else:
        cx, cy = out_nd['x'] - in_nd['x'], out_nd['y'] - in_nd['y']
        waypoints.sort(key=lambda w: (w['x'] - in_nd['x']) * cx
                       + (w['y'] - in_nd['y']) * cy)
    return [in_nd, *waypoints, out_nd]


def extract_rows(nodes):
    """Group topology nodes by row_id into rows, tolerating mixed int/str ids.

    Each row dict carries 'a'/'b' (entry/exit XY, kept for spawn and legacy
    callers) and 'path': the full ordered polyline entry -> waypoints -> exit.
    Curved contour rows therefore keep their shape instead of collapsing to
    the straight entry-exit chord.
    """
    by_rid = {}
    members_by_rid = {}
    for name, nd in nodes.items():
        rid = nd['row_id']
        if rid is None:
            continue
        # row_id's type is inconsistent across sources: get_maize_topo.py's
        # bootstrap YAML writes it as a raw int, while models.py's
        # TopoProperties.row_id is typed str (and UI/F2C-saved nodes follow
        # that) — so a map combining both (e.g. UI-authored rows appended to
        # a bootstrap map) ends up with by_rid keys of mixed int/str types,
        # which sorted() below can't compare. Normalize to int when the
        # value is numeric so "3" and 3 collapse to the same row instead of
        # silently becoming two, falling back to the raw value for any
        # genuinely non-numeric row id.
        try:
            rid = int(rid)
        except (TypeError, ValueError):
            pass
        by_rid.setdefault(rid, {})[nd['row_role'] or name] = nd
        members_by_rid.setdefault(rid, []).append(nd)

    rows = []
    for rid, ends in sorted(by_rid.items(), key=lambda item: (
            0, item[0]) if isinstance(item[0], int) else (1, str(item[0]))):
        in_nd = ends.get('entry')
        out_nd = ends.get('exit')
        if in_nd is None or out_nd is None:
            ins = [v for k, v in ends.items() if str(k).upper().endswith('IN')]
            outs = [v for k, v in ends.items() if str(k).upper().endswith('OUT')]
            in_nd = in_nd or (ins[0] if ins else None)
            out_nd = out_nd or (outs[0] if outs else None)
        if in_nd is None or out_nd is None:
            print(f"  skip row {rid}: need both IN and OUT", file=sys.stderr)
            continue
        chain = _order_row_chain(members_by_rid[rid], in_nd, out_nd)
        # The GPS fix goes with whatever node had one; later we back-solve the
        # lat/lon of topo-frame (0, 0) from that (lat, lon) <-> (x, y) pair.
        if in_nd.get('gps_lat') is not None:
            gps_lat, gps_lon = in_nd['gps_lat'], in_nd['gps_lon']
            gps_x, gps_y = in_nd['x'], in_nd['y']
        elif out_nd.get('gps_lat') is not None:
            gps_lat, gps_lon = out_nd['gps_lat'], out_nd['gps_lon']
            gps_x, gps_y = out_nd['x'], out_nd['y']
        else:
            gps_lat = gps_lon = gps_x = gps_y = None
        rows.append({
            'rid': rid,
            'a': (in_nd['x'], in_nd['y']),
            'b': (out_nd['x'], out_nd['y']),
            'path': [(nd['x'], nd['y']) for nd in chain],
            'gps_lat': gps_lat,
            'gps_lon': gps_lon,
            'gps_x': gps_x,
            'gps_y': gps_y,
        })
    return rows


def read_ground_mesh_extent():
    """Return (min_x, max_x, min_y, max_y) of the custom ground mesh, or None.

    Reads the first OBJ/STL under TERRAIN_UPLOAD_DIR. The ground's own
    coordinates define the placement frame Forest3D uses (they get shifted
    into the topo frame later by patch_world_model_poses), so crops that span
    these extents fill the actual mesh.
    """
    if not TERRAIN_UPLOAD_DIR.is_dir():
        return None
    for p in sorted(TERRAIN_UPLOAD_DIR.iterdir()):
        if p.suffix.lower() == '.obj':
            xs, ys = [], []
            try:
                for line in p.read_text().splitlines():
                    if line.startswith('v '):
                        parts = line.split()
                        if len(parts) >= 3:
                            xs.append(float(parts[1]))
                            ys.append(float(parts[2]))
            except (OSError, ValueError):
                continue
            if xs:
                return min(xs), max(xs), min(ys), max(ys)
        if p.suffix.lower() == '.stl':
            import struct as _struct
            try:
                header = p.read_bytes()
                if len(header) >= 84 and header[:5] not in (b'solid',):
                    _n = _struct.unpack('<I', header[80:84])[0]
                    with p.open('rb') as f:
                        f.seek(84)
                        xs, ys = [], []
                        for _ in range(min(_n, 200000)):
                            raw = f.read(50)
                            if len(raw) < 50:
                                break
                            tri = _struct.unpack('<12f', raw[:48])
                            for i in range(3):
                                xs.append(tri[3 * i])
                                ys.append(tri[3 * i + 1])
                    if xs:
                        return min(xs), max(xs), min(ys), max(ys)
            except (OSError, ValueError, _struct.error, IndexError):
                pass
            text = p.read_text(errors='ignore')
            xs, ys = [], []
            for lineno, line in enumerate(text.splitlines(), 1):
                parts = line.split()
                if len(parts) >= 3 and parts[0] == 'vertex':
                    try:
                        xs.append(float(parts[1]))
                        ys.append(float(parts[2]))
                    except ValueError:
                        sys.exit(
                            f"ERROR: malformed vertex on line {lineno} of "
                            f"{p.name}: {line.strip()!r} — reject and re-upload "
                            "the mesh")
            if xs:
                return min(xs), max(xs), min(ys), max(ys)
    return None


def row_path(row):
    """Return a row's ordered polyline, falling back to the entry-exit chord."""
    return row.get('path') or [row['a'], row['b']]


def path_length(path):
    """Total arc length of a polyline in metres."""
    return sum(math.hypot(q[0] - p[0], q[1] - p[1])
               for p, q in zip(path, path[1:]))


def resample_path(path, spacing):
    """Return (x, y, tangent_yaw) samples every ~spacing m along a polyline.

    Samples always include both end points, with the interval stretched
    slightly so it divides the length evenly. A zero-length path gives one
    sample. This is what puts plants ON the authored row line, whatever its
    bearing or curvature.
    """
    total = path_length(path)
    n = max(1, int(round(total / spacing)) + 1) if total > 0 else 1
    if n == 1:
        yaw = (math.atan2(path[-1][1] - path[0][1], path[-1][0] - path[0][0])
               if len(path) > 1 else 0.0)
        return [(path[0][0], path[0][1], yaw)]
    targets = [total * i / (n - 1) for i in range(n)]
    out, seg, seg_start = [], 0, 0.0
    for tgt in targets:
        while (seg < len(path) - 2 and
               seg_start + math.hypot(path[seg + 1][0] - path[seg][0],
                                      path[seg + 1][1] - path[seg][1]) < tgt):
            seg_start += math.hypot(path[seg + 1][0] - path[seg][0],
                                    path[seg + 1][1] - path[seg][1])
            seg += 1
        p, q = path[seg], path[seg + 1]
        seg_len = math.hypot(q[0] - p[0], q[1] - p[1])
        u = 0.0 if seg_len == 0 else min(1.0, max(0.0, (tgt - seg_start) / seg_len))
        out.append((p[0] + u * (q[0] - p[0]), p[1] + u * (q[1] - p[1]),
                    math.atan2(q[1] - p[1], q[0] - p[0])))
    return out


def order_rows_across(rows):
    """Sort rows by position across the field, whatever its orientation.

    Projects each row's centroid onto the normal of the mean row direction, so
    "middle N rows" means the spatially central strip even for rows at an
    arbitrary bearing (a fixed X/Y axis choice breaks as soon as rows tilt).
    """
    ref = None
    sx = sy = 0.0
    for r in rows:
        dx, dy = r['b'][0] - r['a'][0], r['b'][1] - r['a'][1]
        if ref is None and (dx or dy):
            ref = (dx, dy)
        if ref and dx * ref[0] + dy * ref[1] < 0:  # align IN->OUT sense
            dx, dy = -dx, -dy
        sx += dx
        sy += dy
    norm = math.hypot(sx, sy) or 1.0
    nx, ny = -sy / norm, sx / norm

    def centroid_offset(r):
        pts = row_path(r)
        return (sum(p[0] for p in pts) * nx + sum(p[1] for p in pts) * ny) / len(pts)

    return sorted(rows, key=centroid_offset)


def compute_field_params(rows, headland_width, default_row_width, plant_spacing,
                          min_furrow_width=0.1, ground_extent=None):
    """Derive terrain size, GPS origin and terrain offset from the row paths.

    Plants are placed directly along each row's polyline by
    place_plants_along_rows(), so Forest3D is only asked for a ground mesh.
    That mesh is a rectangle covering every path point plus the headland; it
    is centred on the origin by Forest3D, so the offset that moves it onto the
    paths is simply the path bounding-box centre. No row-axis, spacing or
    orientation assumptions are made.
    """
    if len(rows) < 1:
        sys.exit("ERROR: need at least 1 row")

    pts = [p for r in rows for p in row_path(r)]
    min_x, max_x = min(p[0] for p in pts), max(p[0] for p in pts)
    min_y, max_y = min(p[1] for p in pts), max(p[1] for p in pts)
    path_w, path_h = max_x - min_x, max_y - min_y
    centre_x, centre_y = (min_x + max_x) / 2.0, (min_y + max_y) / 2.0

    # Furrow/row widths only describe the (essentially flat, 1.5 mm) bed
    # Forest3D still builds; the visible plants no longer depend on them.
    row_width = max(default_row_width, 0.2)
    furrow_width = max(min_furrow_width, 0.1)

    headland = max(headland_width, 0.0)
    if ground_extent is not None:
        g_min_x, g_max_x, g_min_y, g_max_y = ground_extent
        field_length, field_width = g_max_x - g_min_x, g_max_y - g_min_y
        if path_w > field_length or path_h > field_width:
            sys.exit(
                f"ERROR: uploaded ground mesh is {field_length:.2f} x "
                f"{field_width:.2f}m but the topo rows span "
                f"{path_w:.2f} x {path_h:.2f}m — the mesh cannot contain "
                "this row layout. Upload a larger mesh or trim the map.")
        # Move the mesh's own centre onto the rows' centre.
        shift_x = centre_x - (g_min_x + g_max_x) / 2.0
        shift_y = centre_y - (g_min_y + g_max_y) / 2.0
    else:
        field_length = path_w + 2 * headland
        field_width = path_h + 2 * headland
        shift_x, shift_y = centre_x, centre_y

    # Plant count for the density / visual-clip decisions below.
    plants = [len(resample_path(row_path(r), plant_spacing)) for r in rows]
    plants_per_row = max(1, round(sum(plants) / len(plants)))
    derived_density = sum(plants)

    # GPS origin: the lat/lon of topo-frame local (0, 0). A GPS tag on a node
    # is the fix recorded at THAT node's (x, y), which is generally not (0,0);
    # gz-sim-navsat-system's <spherical_coordinates> declares the geodetic
    # position of the ORIGIN, so back-solve it with a flat-earth approximation
    # (fine at <1km field scale): 1 deg lat ~= 111320 m, lon ~= that * cos(lat).
    gps_ref = next(((r['gps_lat'], r['gps_lon'], r['gps_x'], r['gps_y'])
                     for r in rows if r['gps_lat'] is not None), None)
    if gps_ref is not None:
        ref_lat, ref_lon, ref_x, ref_y = gps_ref
        m_per_deg_lat = 111320.0
        m_per_deg_lon = 111320.0 * math.cos(math.radians(ref_lat))
        gps_lat = ref_lat - (ref_y / m_per_deg_lat)
        gps_lon = ref_lon - (ref_x / m_per_deg_lon) if m_per_deg_lon else ref_lon
    else:
        # No GPS-tagged node anywhere in this map (synthetic/no-GPS map) —
        # fall back to Field 27's location rather than leaving this None,
        # which upstream would otherwise resolve to gz-sim-navsat-system's
        # own (0, 0) default. See DEFAULT_FIELD_LAT/LON above.
        gps_lat, gps_lon = DEFAULT_FIELD_LAT, DEFAULT_FIELD_LON

    # Forest3D builds int(extent / resolution) cells per side with Python
    # loops, so keep very large fields to a bounded grid.
    resolution = max(0.1, min(0.25, row_width / 2.5),
                     max(field_length, field_width) / 400.0)

    return {
        'field_length': round(max(field_length, 5.0), 2),
        'field_width': round(max(field_width, 5.0), 2),
        # Forest3D must build >= 1 bed; plants are placed by us, not by it.
        'num_rows': 1,
        'row_width': round(row_width, 3),
        'furrow_width': round(furrow_width, 3),
        'headland_width': headland if ground_extent is None else 0.0,
        'row_height': 0.0015,
        'row_profile': 'rounded',
        'plant_spacing': plant_spacing,
        'stagger': 0.0,
        'resolution': resolution,
    }, gps_lat, gps_lon, plants_per_row, derived_density, (
        round(shift_x, 4),
        round(shift_y, 4),
    )


# Soil assets imported from the NiceGUI cockpit land here (a host dir mounted
# at /workspace/uploads, persisted across image rebuilds). Only the image maps
# are kept; Forest3D classifies them by filename when it builds the ground SDF.
SOIL_UPLOAD_DIR = Path('/workspace/uploads/soil_custom/textures')
_IMG_EXTS = ('.jpg', '.jpeg', '.png')

# Forest3D emits this flat gray material when no texture is found;
# FLAT_SOIL_MATERIAL is the no-asset fallback (plain soil-brown, no files).
GRAY_MATERIAL = (
    '<ambient>0.6 0.6 0.6 1</ambient>\n'
    '                    <diffuse>0.8 0.8 0.8 1</diffuse>'
)
FLAT_SOIL_MATERIAL = (
    '<ambient>0.25 0.15 0.07 1</ambient>\n'
    '                    <diffuse>0.40 0.25 0.12 1</diffuse>\n'
    '                    <specular>0.02 0.02 0.02 1</specular>'
)


def seed_ground_textures(base_dir):
    """Copy imported soil maps into the ground model's texture dir.

    Staged BEFORE `forest3d terrain crop_rows` so Forest3D finds them and
    emits a PBR <albedo_map>/<normal_map>/<roughness_map> material itself.
    Returns the number of image files staged (0 if nothing imported).
    """
    if not SOIL_UPLOAD_DIR.is_dir():
        return 0
    imgs = [p for p in SOIL_UPLOAD_DIR.iterdir()
            if p.suffix.lower() in _IMG_EXTS]
    if not imgs:
        return 0
    dest = base_dir / 'models' / 'ground' / 'texture'
    dest.mkdir(parents=True, exist_ok=True)
    import shutil as _shutil
    for src in imgs:
        _shutil.copy2(src, dest / src.name)
    return len(imgs)


# Default custom terrain meshes imported via the NiceGUI cockpit / host drops
# land here (persisted host dir, bind-mounted at /workspace/uploads). Forest3D
# regenerates models/ground/mesh/terrain.{obj,stl} on every `terrain crop_rows`
# run, so these are restaged over the fresh mesh right afterwards — that is
# what guarantees a user-supplied my_model.obj/stl always shows up in the sim.
TERRAIN_UPLOAD_DIR = Path('/workspace/uploads/terrain')


def restage_ground_terrain(base_dir):
    """Overwrite the Forest3D-generated ground mesh with the custom one.

    Runs right after `forest3d terrain crop_rows`; restages terrain.{obj,stl}
    from /workspace/uploads/terrain back into <base>/models/ground/mesh/ so the
    sim's visual and collision match the user's terrain instead of Forest3D's.
    Returns the number of files restaged (0 = keeping Forest3D's mesh).
    """
    src_dir = TERRAIN_UPLOAD_DIR
    if not src_dir.is_dir():
        print(f"No custom terrain staged at {src_dir} — keeping Forest3D's mesh")
        return 0
    meshes = [p for p in src_dir.iterdir()
              if p.suffix.lower() in ('.obj', '.stl')]
    if not meshes:
        print(f"No mesh (.obj/.stl) in {src_dir} — keeping Forest3D's mesh")
        return 0

    mesh_dir = base_dir / 'models' / 'ground' / 'mesh'
    mesh_dir.mkdir(parents=True, exist_ok=True)
    import shutil as _shutil

    restaged = 0
    for src in sorted(meshes):
        dst = mesh_dir / f'terrain{src.suffix.lower()}'
        _shutil.copy2(src, dst)
        restaged += 1
        print(f"Restaged custom ground mesh {src.name} -> {dst}")
    return restaged


def patch_ground_soil_colour(base_dir):
    """Recolour Forest3D's flat-gray ground material to soil-brown.

    Only used as the no-asset fallback; when textures were seeded Forest3D has
    already written a PBR material and this is skipped. Warns (rather than
    silently no-op'ing) if the gray block can't be found, so material drift in
    Forest3D is visible instead of leaving the ground concrete-gray.
    """
    ground_sdf = base_dir / 'models' / 'ground' / 'model.sdf'
    if not ground_sdf.exists():
        print(f"WARNING: {ground_sdf} not found — skipping soil colour patch",
              file=sys.stderr)
        return
    sdf_text = ground_sdf.read_text()
    if GRAY_MATERIAL not in sdf_text:
        print(f"WARNING: expected gray material block not found in {ground_sdf}; "
              "Forest3D may have changed its default material — leaving as-is",
              file=sys.stderr)
        return
    ground_sdf.write_text(sdf_text.replace(GRAY_MATERIAL, FLAT_SOIL_MATERIAL))
    print(f"Patched {ground_sdf} with flat soil colour (no asset imported)")


# Forest3D's generated ground <collision> has no <surface><friction> block;
# with zero friction the track belts have nothing to grip and /odom stays at
# the noise floor. These strings bracket the one fixed block every generated
# ground model.sdf is expected to contain.
GROUND_COLLISION_GEOMETRY_CLOSE = (
    '                <geometry>\n'
    '                    <mesh>\n'
    '                        <uri>model://ground/mesh/terrain.stl</uri>\n'
    '                    </mesh>\n'
    '                </geometry>\n'
    '            </collision>'
)
GROUND_FRICTION_SURFACE = (
    '                <geometry>\n'
    '                    <mesh>\n'
    '                        <uri>model://ground/mesh/terrain.stl</uri>\n'
    '                    </mesh>\n'
    '                </geometry>\n'
    '                <surface>\n'
    '                    <friction>\n'
    '                        <ode>\n'
    '                            <mu>0.9</mu>\n'
    '                            <mu2>0.9</mu2>\n'
    '                        </ode>\n'
    '                    </friction>\n'
    '                    <contact>\n'
    '                        <ode/>\n'
    '                    </contact>\n'
    '                </surface>\n'
    '            </collision>'
)


def patch_ground_friction(base_dir):
    """Add a friction surface to the ground collision so tracks can grip.

    Must run every time, same as patch_world_physics_engine, since Forest3D
    regenerates model.sdf from scratch on every `forest3d terrain crop_rows`
    run. Warns (rather than silently no-op'ing) if the expected collision
    block isn't found, so a Forest3D template change becomes visible instead
    of leaving a frictionless, non-functional ground.
    """
    ground_sdf = base_dir / 'models' / 'ground' / 'model.sdf'
    if not ground_sdf.exists():
        print(f"WARNING: {ground_sdf} not found — skipping ground friction patch",
              file=sys.stderr)
        return
    sdf_text = ground_sdf.read_text()
    if '<surface>' in sdf_text:
        print(f"{ground_sdf} already has a <surface> block — nothing to patch")
        return
    if GROUND_COLLISION_GEOMETRY_CLOSE not in sdf_text:
        print(f"WARNING: expected ground collision block not found in "
              f"{ground_sdf}; Forest3D may have changed its template — "
              "leaving as-is. TrackedVehicle needs ground friction; verify "
              "manually.", file=sys.stderr)
        return
    sdf_text = sdf_text.replace(GROUND_COLLISION_GEOMETRY_CLOSE,
                                 GROUND_FRICTION_SURFACE)
    ground_sdf.write_text(sdf_text)
    print(f"Patched {ground_sdf} with mu=mu2=0.9 friction surface "
          "(required for TrackedVehicle/TrackController to grip the ground)")


# Forest3D's generated world uses type="ignored"/type='ode' (Gazebo's
# compiled-in ODE default), but the track drive needs DART. Match either
# quoting so template changes can't silently no-op the patch.
_PHYSICS_TYPE_RE = re.compile(r'(<physics\b[^>]*\btype=)(["\'])[^"\']*\2')
DART_TYPE = "dartsim"


def patch_world_physics_engine(world_path):
    """Switch the generated world's physics engine to DART.

    Warns if no <physics> tag is found so Forest3D template changes become
    visible instead of silently leaving ODE physics.
    """
    world_path = Path(world_path)
    if not world_path.exists():
        print(f"WARNING: {world_path} not found — skipping physics engine patch",
              file=sys.stderr)
        return
    world_text = world_path.read_text()
    match = _PHYSICS_TYPE_RE.search(world_text)
    if not match:
        print(f"WARNING: no <physics type=...> tag found in {world_path}; "
              "Forest3D may have changed its template — leaving as-is. "
              "TrackedVehicle/TrackController require DART; verify manually.",
              file=sys.stderr)
        return
    current_type = world_text[match.start():match.end()]
    if f'type={match.group(2)}{DART_TYPE}{match.group(2)}' in current_type:
        print(f"{world_path} already uses {DART_TYPE} physics — nothing to patch")
        return
    patched_text = _PHYSICS_TYPE_RE.sub(
        lambda m: f'{m.group(1)}{m.group(2)}{DART_TYPE}{m.group(2)}',
        world_text, count=1)
    world_path.write_text(patched_text)
    print(f"Patched {world_path}: physics -> {DART_TYPE} "
          "(required for TrackedVehicle/TrackController motion)")


def patch_world_spherical_coordinates(world_path, gps_lat, gps_lon):
    """Insert a <spherical_coordinates> block so gz-sim-navsat-system reports
    real WGS84 fixes.

    Forest3D's world template has none; without it every fix is a frozen
    (0,0) that fusioncore's outlier gate rejects. gps_lat/gps_lon must be the
    geodetic position of topo-frame (0,0) — back-solved by compute_field_params,
    NOT a raw node GPS tag.
    """
    world_path = Path(world_path)
    if not world_path.exists():
        print(f"WARNING: {world_path} not found — skipping spherical "
              "coordinates patch", file=sys.stderr)
        return
    if gps_lat is None or gps_lon is None:
        print("WARNING: no GPS origin found in topo map — skipping "
              "spherical coordinates patch. gz-sim-navsat-system will "
              "default to (0,0) and every /gnss/fix will be rejected as "
              "out of range.", file=sys.stderr)
        return

    world_text = world_path.read_text()
    if '<spherical_coordinates>' in world_text:
        print(f"{world_path} already has <spherical_coordinates> — nothing to patch")
        return

    match = re.search(r'(<world\s+name="[^"]*"[^>]*>)', world_text)
    if not match:
        print(f"WARNING: no <world name=...> tag found in {world_path}; "
              "Forest3D may have changed its template — leaving as-is. "
              "gz-sim-navsat-system will default to (0,0). Verify manually.",
              file=sys.stderr)
        return

    block = (
        '\n    <spherical_coordinates>\n'
        '      <surface_model>EARTH_WGS84</surface_model>\n'
        '      <world_frame_orientation>ENU</world_frame_orientation>\n'
        f'      <latitude_deg>{gps_lat:.8f}</latitude_deg>\n'
        f'      <longitude_deg>{gps_lon:.8f}</longitude_deg>\n'
        '      <elevation>0</elevation>\n'
        '      <heading_deg>0</heading_deg>\n'
        '    </spherical_coordinates>'
    )
    patched = world_text[:match.end()] + block + world_text[match.end():]
    world_path.write_text(patched)
    print(f"Patched {world_path}: <spherical_coordinates> datum "
          f"({gps_lat:.6f}, {gps_lon:.6f}) -> gz-sim-navsat-system will "
          "now report real fixes instead of a frozen (0,0)")


def patch_world_model_poses(world_path, terrain_offset):
    """Shift ALL spawned models by terrain_offset so the Forest3D world aligns
    with the topo nav coordinate frame.

    Terrain and crop models are siblings in the Entity Tree (not parent/child),
    so moving only the ground leaves crops stranded at origin — every <include>
    pose must be shifted. terrain_offset is (x, y): corrected topo centroid
    minus the row_width/2 local-frame bias (see compute_field_params).
    """
    import xml.etree.ElementTree as _ET

    world_path = Path(world_path)
    if not world_path.exists():
        print(f"WARNING: {world_path} not found — skipping model pose patch",
              file=sys.stderr)
        return

    dx, dy = terrain_offset
    if abs(dx) < 1e-4 and abs(dy) < 1e-4:
        print("Model pose patch: topo centroid already at origin — no shift needed")
        return

    try:
        tree = _ET.parse(world_path)
        root = tree.getroot()
    except _ET.ParseError as exc:
        print(f"WARNING: cannot parse {world_path} as XML — skipping: {exc}",
              file=sys.stderr)
        return

    world_elem = root.find("world") or root
    count = 0
    for include in world_elem.findall("include"):
        pose_elem = include.find("pose")
        if pose_elem is None:
            existing = [0.0] * 6
        else:
            parts = (pose_elem.text or "").split()
            existing = [float(v) for v in parts] + [0.0] * (6 - len(parts))
        new_pose = f"{existing[0]+dx:.6f} {existing[1]+dy:.6f} {existing[2]:.6f} {existing[3]:.6f} {existing[4]:.6f} {existing[5]:.6f}"
        if pose_elem is None:
            pose_elem = _ET.SubElement(include, "pose")
        pose_elem.text = new_pose
        count += 1

    if count == 0:
        print(f"WARNING: no <include> blocks found in {world_path} — nothing patched",
              file=sys.stderr)
        return

    xml_str = _ET.tostring(root, encoding="unicode")
    world_path.write_text('<?xml version="1.0" ?>\n' + xml_str)
    print(f"Patched {world_path}: shifted {count} model(s) by "
          f"({dx:.4f}, {dy:.4f}) to align Forest3D world with topo nav frame")


def snap_plants_to_terrain(world_path, mesh_path):
    """Push every plant (crop/weed/irrigation) onto the ground mesh surface.

    Forest3D bakes crop/weed z from its own terrain; a custom mesh (e.g. from
    agri_field_to_terrain.py) sits at a different height, leaving plants to
    hover or sink. Re-sample the mesh's height at each plant and rewrite its
    z. Runs after patch_world_model_poses so plants are in world frame.
    """
    import struct

    import numpy as np
    import xml.etree.ElementTree as _ET

    world_path = Path(world_path)
    mesh_path = Path(mesh_path)
    if not world_path.exists() or not mesh_path.exists():
        print(f"WARNING: world/mesh missing ({world_path}, {mesh_path}) — "
              "skipping plant snap", file=sys.stderr)
        return

    with open(mesh_path, 'rb') as f:
        data = f.read()
    n_tris = struct.unpack('<I', data[80:84])[0]
    per_tri = 50  # normal(12B) + 3 verts(36B) + attribute(2B)
    if len(data) < 84 or len(data) - 84 != n_tris * per_tri:
        raise ValueError(f"STL header mismatch for {mesh_path}: header says "
                         f"{n_tris} tris but file has {(len(data) - 84) // 50}")
    dt = np.dtype([('normal', '<f4', 3),
                   ('vertex', '<f4', (3, 3)),
                   ('attr', '<u2')])
    tris = np.frombuffer(data, dtype=dt, count=n_tris, offset=84)
    verts = tris['vertex'].reshape(-1, 3)

    from scipy.interpolate import LinearNDInterpolator
    interp = LinearNDInterpolator(verts[:, :2], verts[:, 2])

    try:
        root = _ET.parse(world_path).getroot()
    except _ET.ParseError as exc:
        print(f"WARNING: cannot parse {world_path} — skipping plant snap: {exc}",
              file=sys.stderr)
        return

    world_elem = root.find("world") or root
    # Terrain model pose (x, y, ...) — world coords of the mesh origin.
    tx = ty = 0.0
    for include in world_elem.findall("include"):
        uri = include.findtext("uri") or ""
        if "model://ground" in uri or "ground" in uri:
            pose = (include.findtext("pose") or "0 0 0").split()
            if len(pose) >= 2:
                tx, ty = float(pose[0]), float(pose[1])
            break

    moved = total = 0
    for include in world_elem.findall("include"):
        uri = include.findtext("uri") or ""
        if not uri.startswith("model://"):
            continue
        if any(cat in uri for cat in ("ground", "terrain")):
            continue
        pose = include.findtext("pose") or "0 0 0"
        parts = pose.split()
        if len(parts) < 3:
            continue
        total += 1
        x, y = float(parts[0]) - tx, float(parts[1]) - ty  # mesh-local coords
        z_surface = float(interp(x, y))
        if np.isnan(z_surface):
            print(f"WARNING: plant at mesh-local ({x:.2f}, {y:.2f}) is off the "
                  f"terrain mesh — leaving z as-is", file=sys.stderr)
            continue
        parts[2] = f"{z_surface:.4f}"
        include.find("pose").text = " ".join(parts)
        moved += 1

    world_path.write_text('<?xml version="1.0" ?>\n'
                          + _ET.tostring(root, encoding="unicode"))
    print(f"Snapped {moved}/{total} plant(s) onto the terrain surface "
          f"(mesh {mesh_path.name}, origin {tx:.3f},{ty:.3f})")


def _model_variants(models_path, category, fallback):
    """Return model directory names under models/<category>/ (or [fallback])."""
    if models_path:
        root = Path(models_path) / category
        if root.is_dir():
            found = sorted(d.name for d in root.iterdir()
                           if (d / 'model.sdf').exists())
            if found:
                return found
    return [fallback]


def place_plants_along_rows(world_path, rows, plant_spacing, crop_model,
                            weed_pct, models_path=None, seed=0, jitter=0.02,
                            weed_radius=0.8):
    """Write crop and weed <include>s straight along the topo row polylines.

    Forest3D's own row placement can only draw straight, evenly spaced rows
    along X, so it cannot follow rows a farmer authored in the field (any
    bearing, uneven spacing, curves). Instead: drop whatever crops/weeds
    Forest3D put in the world, resample each row's entry -> waypoints -> exit
    polyline every ~plant_spacing m, and write one crop per sample in topo
    (world) coordinates. Must run AFTER patch_world_model_poses (which shifts
    every include) so these poses are not moved a second time.

    Weeds scatter around random crops: weed_pct is a percentage of the crop
    count times WEEDS_PER_CROP, as before. Seeded, so the same inputs always
    give the same world (worldgen.sh caches on its inputs).
    """
    import random
    import xml.etree.ElementTree as _ET

    world_path = Path(world_path)
    if not world_path.exists():
        print(f"WARNING: {world_path} not found — no plants placed",
              file=sys.stderr)
        return 0
    try:
        tree = _ET.parse(world_path)
    except _ET.ParseError as exc:
        print(f"WARNING: cannot parse {world_path} — no plants placed: {exc}",
              file=sys.stderr)
        return 0
    root = tree.getroot()
    world_elem = root.find("world")
    if world_elem is None:
        world_elem = root

    for include in list(world_elem.findall("include")):
        uri = include.findtext("uri") or ""
        if uri.startswith(("model://crop/", "model://weed/")):
            world_elem.remove(include)

    rng = random.Random(seed)

    def add(category, variant, idx, x, y, yaw):
        inc = _ET.SubElement(world_elem, "include")
        _ET.SubElement(inc, "uri").text = f"model://{category}/{variant}"
        _ET.SubElement(inc, "name").text = f"{category}_{idx}"
        _ET.SubElement(inc, "pose").text = (
            f"{x:.4f} {y:.4f} 0.0000 0.0000 0.0000 {yaw:.4f}")

    crops = []
    for row in rows:
        for x, y, _tangent in resample_path(row_path(row), plant_spacing):
            crops.append((x + rng.uniform(-jitter, jitter),
                          y + rng.uniform(-jitter, jitter)))
    for i, (x, y) in enumerate(crops):
        add("crop", crop_model, i, x, y, rng.uniform(0.0, 2 * math.pi))

    n_weeds = int(len(crops) * WEEDS_PER_CROP * weed_pct / 100.0) if crops else 0
    weed_variants = _model_variants(models_path, 'weed', 'weed1')
    for i in range(n_weeds):
        cx, cy = crops[rng.randrange(len(crops))]
        r = rng.uniform(0.2, max(0.3, weed_radius))
        a = rng.uniform(0.0, 2 * math.pi)
        add("weed", rng.choice(weed_variants), i,
            cx + r * math.cos(a), cy + r * math.sin(a),
            rng.uniform(0.0, 2 * math.pi))

    world_path.write_text('<?xml version="1.0" ?>\n'
                          + _ET.tostring(root, encoding="unicode"))
    print(f"Placed {len(crops)} crop(s) and {n_weeds} weed(s) along "
          f"{len(rows)} topo row path(s)")
    return len(crops)


def patch_crop_model_uri(world_path, model_name):
    """Replace the URI in model://crop/ includes with the selected model.

    Forest3D randomly picks a subdirectory from models/crop/; this forces all
    crop includes to point at the user-selected model instead.
    """
    import xml.etree.ElementTree as _ET

    world_path = Path(world_path)
    if not world_path.exists():
        print(f"WARNING: {world_path} not found — skipping crop model patch",
              file=sys.stderr)
        return

    try:
        root = _ET.parse(world_path).getroot()
    except _ET.ParseError as exc:
        print(f"WARNING: cannot parse {world_path} — skipping model patch: {exc}",
              file=sys.stderr)
        return

    world_elem = root.find("world") or root
    changed = 0
    for include in world_elem.findall("include"):
        uri_elem = include.find("uri")
        if uri_elem is not None and (uri_elem.text or "").startswith("model://crop/"):
            uri_elem.text = f"model://crop/{model_name}"
            changed += 1

    if changed == 0:
        print(f"WARNING: no model://crop/ includes found in {world_path} — nothing patched",
              file=sys.stderr)
        return

    world_path.write_text('<?xml version="1.0" ?>\n'
                          + _ET.tostring(root, encoding="unicode"))
    print(f"Patched {changed} crop include(s) to model://crop/{model_name}")


def _read_mesh_scales(model_sdf_path):
    """Return list of (section_tag, scale_text) for each <mesh><scale> found."""
    import xml.etree.ElementTree as _ET
    scales = []
    root = _ET.parse(model_sdf_path).getroot()
    for section in root.iter("visual"):
        geom = section.find("geometry")
        if geom is not None:
            mesh = geom.find("mesh")
            if mesh is not None:
                scale_el = mesh.find("scale")
                if scale_el is not None:
                    scales.append((section.tag, scale_el.text))
    for section in root.iter("collision"):
        geom = section.find("geometry")
        if geom is not None:
            mesh = geom.find("mesh")
            if mesh is not None:
                scale_el = mesh.find("scale")
                if scale_el is not None:
                    scales.append((section.tag, scale_el.text))
    return scales


def _model_sdf_path(models_path, category, model_name):
    return Path(models_path) / category / model_name / "model.sdf"


_BASE_SCALE_FILE = ".base_scale"


def _get_base_scales(model_sdf_path):
    """Return (base_scales dict, was_initialised)."""
    base_file = model_sdf_path.parent / _BASE_SCALE_FILE
    if base_file.exists():
        data = {}
        for line in base_file.read_text().strip().splitlines():
            if "=" in line:
                k, v = line.split("=", 1)
                data[k.strip()] = float(v.strip())
        return data, False
    scales = _read_mesh_scales(model_sdf_path)
    data = {}
    for tag, text in scales:
        vals = text.strip().split()
        if vals:
            data[tag] = float(vals[0])
    base_file.write_text("\n".join(f"{k}={v}" for k, v in data.items()) + "\n")
    return data, True


def patch_model_mesh_scale(models_path, category, model_name, user_factor):
    """Scale the mesh <scale> in model.sdf so Gazebo renders it correctly.

    Gazebo Sim ignores <include><scale> in world files (gz-sim#195); the only
    reliable resize is patching the mesh <scale>. The *base* scale (original
    design value) is persisted in a .base_scale file beside model.sdf so
    repeated runs compose: new = base * user_factor.
    """
    import xml.etree.ElementTree as _ET

    sdf_path = _model_sdf_path(models_path, category, model_name)
    if not sdf_path.exists():
        print(f"WARNING: {sdf_path} not found — skipping mesh scale patch",
              file=sys.stderr)
        return

    base_scales, _ = _get_base_scales(sdf_path)
    if not base_scales:
        print(f"WARNING: no <mesh><scale> found in {sdf_path} — nothing to patch",
              file=sys.stderr)
        return

    try:
        tree = _ET.parse(sdf_path)
        root = tree.getroot()
    except _ET.ParseError as exc:
        print(f"WARNING: cannot parse {sdf_path} — skipping mesh scale patch: {exc}",
              file=sys.stderr)
        return

    _section_tags = ["visual", "collision"]
    changed = 0
    for tag in _section_tags:
        for section in root.iter(tag):
            geom = section.find("geometry")
            if geom is not None:
                mesh = geom.find("mesh")
                if mesh is not None:
                    scale_el = mesh.find("scale")
                    if scale_el is not None:
                        base = base_scales.get(tag)
                        if base is None:
                            vals = scale_el.text.strip().split()
                            base = float(vals[0]) if vals else 1.0
                        new_val = base * user_factor
                        scale_el.text = f"{new_val:.6f} {new_val:.6f} {new_val:.6f}"
                        changed += 1

    if changed == 0:
        print(f"WARNING: no <mesh><scale> elements found in {sdf_path}",
              file=sys.stderr)
        return

    sdf_path.write_text('<?xml version="1.0" ?>\n'
                        + _ET.tostring(root, encoding="unicode"))
    print(f"Patched {changed} mesh scale(s) in {sdf_path} "
          f"(base × {user_factor:.3f})")


def write_spawn_pose(x, y, z, output_path):
    """Write the robot spawn pose so sim.launch.py can read it at startup."""
    Path(output_path).write_text(f"{x} {y} {z}\n")
    print(f"Wrote spawn pose ({x}, {y}, {z}) to {output_path}")


def write_forest3d_yaml(params, gps_lat, gps_lon, output_path, density_crop,
                        model_path, density_weed=0, settings=None):
    density = {'crop': density_crop}
    # Forest3D's crop_rows places a category only if it has a density entry
    # (placement does density_config.get(category, 0)). The weed/ models are
    # ClusteredPlacement, anchored near crops — omit/zero to get a clean field.
    if density_weed:
        density['weed'] = density_weed
    config = {
        'terrain': {
            'type': 'crop_rows',
            'scale_factor': 1.0,
            'material_name': 'Terrain/Soil',
            'crop_rows': params,
        },
        'density': density,
        'paths': {},
    }
    if settings:
        # User-facing placement inputs, echoed back so callers that regenerate
        # the world later (e.g. sowbot_sim.launch.py's world_gen step) can
        # replay the exact same generation instead of falling back to defaults.
        config['settings'] = settings
    if model_path:
        config['paths']['models_path'] = str(model_path)

    # GPS origin for georeferencing (used by Forest3D if configured).
    Path(output_path).write_text(yaml.dump(config, default_flow_style=False, sort_keys=False))
    print(f"Wrote {output_path}")
    print(f"  Rows: {params['num_rows']}, Length: {params['field_length']}m, "
          f"Width: {params['field_width']}m")
    print(f"  Row width: {params['row_width']}m, Furrow: {params['furrow_width']}m")
    if gps_lat is not None:
        print(f"  GPS origin: ({gps_lat:.6f}, {gps_lon:.6f})")


if __name__ == '__main__':
    ap = argparse.ArgumentParser(
        description='Convert Sowbot topo map to Forest3D config & world')
    ap.add_argument('--topo', default='/workspace/maps/maize_map',
                    help='Saved topo map file (YAML or JSON)')
    ap.add_argument('--out', default='forest3d.yaml',
                    help='Output forest3d.yaml path')
    ap.add_argument('--headland', type=float, default=2.0,
                    help='Headland width in metres')
    ap.add_argument('--row-width', type=float, default=0.9,
                    help='Default crop row width (m) if spacing cannot be '
                         'measured from the data')
    ap.add_argument('--min-furrow-width', type=float, default=0.1,
                    help='Smallest furrow width (m) to accept before the '
                         '40%%-of-spacing auto-correction kicks in. '
                         "Forest3D's hard floor is 0.1 m regardless.")
    ap.add_argument('--plant-spacing', type=float, default=1.0,
                    help='Spacing between plants along a row (m). Forest3D '
                         'places int(row_length / plant_spacing) plants per row.')
    ap.add_argument('--plant-scale', type=float, default=1.0,
                    help='Uniform scale factor applied to selected category '
                         '(default: 1.0). Multiplies the model visual size '
                         'without affecting spacing or density.')
    ap.add_argument('--scale-category', type=str, default='all',
                    help='Category to scale ("all", "crop", "weed", '
                         '"irrigation"). Defaults to "all" which scales '
                         'every model:// include.')
    ap.add_argument('--crop-model', type=str, default='plant',
                    help='Crop model variant name to use (default: plant). '
                         'Must be a subdirectory under models/crop/ containing '
                         'a valid Gazebo model (model.config + model.sdf).')
    ap.add_argument('--density', type=int, default=None,
                    help='Only used to cap the crop count that scales the weed '
                         'count; crops are placed one per --plant-spacing '
                         'along every row path regardless.')
    ap.add_argument('--models-path', type=str, default=None,
                    help='Path to Forest3D models directory')
    ap.add_argument('--weed-density', type=int, default=None,
                    help='Weed density as a percentage of the crop count '
                         '(0-100): 100% scatters WEEDS_PER_CROP weeds per '
                         f'crop plant (currently {WEEDS_PER_CROP}). Defaults '
                         'to ~65%% of the crop count; 0 disables weeds for a '
                         'clean field.')
    ap.add_argument('--generate', action='store_true',
                    help='Also run forest3d generate after writing config')
    ap.add_argument('--world-out', default=None,
                    help='Output world path for forest3d generate (optional)')
    ap.add_argument('--spawn-out', default='/workspace/spawn_pose.txt',
                    help='Output path for the robot spawn pose (x y z), '
                         'read by sim.launch.py')
    ap.add_argument('--spawn-z', type=float, default=1.0,
                    help='Spawn height (base_footprint), as a world-Z drop '
                         'point rather than a ground-level offset — this '
                         'script does not sample the generated terrain\'s '
                         'actual elevation at (x, y), so a near-zero value '
                         'spawns the robot embedded in any non-flat terrain. '
                         'A ~1.0m drop lets gravity settle it onto the mesh '
                         'regardless of terrain shape at the spawn point.')
    args = ap.parse_args()

    nodes = load_topo(args.topo)
    rows = extract_rows(nodes)
    if not rows:
        sys.exit("ERROR: no complete rows found in topo map")

    print(f"Loaded {len(rows)} rows from {args.topo}")

    nav_rows = rows  # keep the full set for spawn-point lookup below

    # Forest3D's hard schema floor is 0.1 — clamp the requested minimum up to
    # that so a caller who passes e.g. 0.0 (meaning "as narrow as possible")
    # gets the actual achievable floor, rather than something that would
    # still bounce off the inner clamp in compute_field_params for the wrong
    # reason.
    min_furrow_width = max(args.min_furrow_width, 0.1)

    ground_extent = read_ground_mesh_extent()
    if ground_extent is not None:
        gx0, gx1, gy0, gy1 = ground_extent
        print(f"Custom ground mesh {gx1-gx0:.1f} x {gy1-gy0:.1f} m — planting to cover it")

    # Terrain geometry (field_length/width, GPS origin, terrain_offset) is
    # always derived from the COMPLETE row set. The simulator uses nav_rows
    # for navigation/spawn selection regardless of how many rows get a
    # visual crop model below, so sizing the mesh to a clipped subset would
    # leave outer real rows physically outside the generated ground plane.
    params, gps_lat, gps_lon, plants_per_row, derived_density, terrain_offset = compute_field_params(
        nav_rows, args.headland, args.row_width, args.plant_spacing,
        min_furrow_width=min_furrow_width, ground_extent=ground_extent)

    # Rendering every row of a real, F2C-planned full-field coverage plan
    # is both unnecessary (this is just for a Gazebo visual) and can blow
    # through Forest3D's schema caps (FOREST3D_MAX_ROWS/DENSITY) — either
    # because there are simply too many rows, or because a small row count
    # still exceeds FOREST3D_MAX_DENSITY at tight plant_spacing. Clip to a
    # small visualization-only subset whenever either cap is exceeded —
    # this ONLY changes params['num_rows'] (how many crop-row models get
    # placed); field dimensions/terrain_offset/GPS origin stay bound to
    # nav_rows above, and maps/maize_map (real navigation, nav_rows/
    # find_spawn_node) is never touched or re-saved by this script.
    if len(rows) > VISUAL_TARGET_ROWS or derived_density > FOREST3D_MAX_DENSITY:
        max_by_density = max(1, FOREST3D_MAX_DENSITY // max(plants_per_row, 1))
        max_visual_rows = min(
            len(rows), VISUAL_TARGET_ROWS, FOREST3D_MAX_ROWS, max_by_density)
        # Order rows by position ACROSS the field at any bearing (a fixed
        # X/Y axis choice mis-sorts tilted rows), and take the contiguous
        # middle strip. Plants are written per row below, so the rows drawn
        # are exactly these ones, not whichever Forest3D starts counting from.
        rows_by_centre = order_rows_across(rows)
        start = (len(rows_by_centre) - max_visual_rows) // 2
        rows = rows_by_centre[start:start + max_visual_rows]
        print(f"  NOTE: field has {len(nav_rows)} real rows (density "
              f"{derived_density}) — only rendering the middle {len(rows)} "
              f"rows in Gazebo (VISUAL_TARGET_ROWS={VISUAL_TARGET_ROWS}, "
              f"FOREST3D_MAX_DENSITY={FOREST3D_MAX_DENSITY}). Navigation "
              f"(maps/maize_map) keeps all {len(nav_rows)} rows, and the "
              f"generated terrain still spans the full field — this only "
              f"limits how many rows get a visual crop model.")

    # Robot spawn point: the topo map's HOME node (or first row's IN node as
    # fallback), shifted by the same terrain_offset that's about to be baked
    # into the world. Without this, the robot spawns at literal world (0,0)
    # (sim.launch.py's hardcoded default) while the terrain — and HOME's
    # *intended* position relative to it — has moved by terrain_offset.
    # Before terrain_offset existed this was harmless because nothing in the
    # world ever moved; once the world is shifted, HOME and the hardcoded
    # spawn silently drift apart.
    spawn_topo_x, spawn_topo_y = find_spawn_node(nodes, nav_rows)
    # NOTE: topo node coordinates are ALREADY in world/map frame — that's the
    # entire point of a topo map. terrain_offset is the correction applied to
    # Forest3D's own internally-generated LOCAL mesh coordinates (which are
    # centred on the mesh's own origin) to shift the terrain INTO the topo
    # frame. Adding terrain_offset to a topo node's coordinate double-shifts
    # it: it moves a point that was already correctly placed, by an offset
    # that was only ever meant to apply to the terrain mesh itself. Confirmed
    # against real F2C-authored topo data: F2C_R1_IN at (7.373, -8.246) sits
    # squarely inside the field's real row band; adding terrain_offset_cross
    # (-5.696) pushed it to y=-13.942, ~3.25m past the generated terrain's
    # actual south edge — i.e. off the mesh into open space.
    spawn_world_x, spawn_world_y = round(spawn_topo_x, 4), round(spawn_topo_y, 4)

    # Crops actually drawn: one per ~plant_spacing along every drawn row path.
    # (Forest3D itself is only asked for the ground mesh, so this number is
    # informational and feeds the weed count; it is never given to Forest3D.)
    density = sum(len(resample_path(row_path(r), args.plant_spacing))
                  for r in rows)
    if args.density is not None:
        density = min(density, args.density)
    print(f"  Crops: {density} along {len(rows)} row path(s)")

    weed_density = (int(density * WEEDS_PER_CROP * (args.weed_density / 100))
                    if args.weed_density is not None
                    else int(density * 0.10 * WEEDS_PER_CROP))
    print(f"  Weed density: {weed_density} "
          f"({args.weed_density if args.weed_density is not None else 10}% of "
          f"{density} crops at {WEEDS_PER_CROP}x)")

    write_forest3d_yaml(params, gps_lat, gps_lon, args.out, 1,
                        args.models_path, 0,
                        settings={
                            'plant_spacing': args.plant_spacing,
                            'row_width': args.row_width,
                            'plant_scale': args.plant_scale,
                            'scale_category': args.scale_category,
                            'crop_model': args.crop_model,
                            'weed_density': (args.weed_density
                                             if args.weed_density is not None
                                             else 10),
                        })
    write_spawn_pose(spawn_world_x, spawn_world_y, args.spawn_z, args.spawn_out)

    if args.generate:
        base_cmd = ['python3', '-m', 'forest3d']
        base_dir = Path(args.out).parent

# Step 1a: Stage imported soil maps BEFORE generating so Forest3D
        # writes a PBR material referencing them itself.
        seeded = seed_ground_textures(base_dir)
        if seeded:
            print(f"Staged {seeded} soil texture map(s) for the ground model")

        # Step 1b: Generate the crop-row terrain mesh (models/ground/).
        terrain_cmd = [*base_cmd, 'terrain', 'crop_rows']
        print(f"Running: {' '.join(terrain_cmd)}")
        subprocess.run(terrain_cmd, check=True)

        # Step 1b.x: Restage a custom mesh over Forest3D's fresh one BEFORE
        # snapping so snap_plants_to_terrain samples THIS mesh.
        if restage_ground_terrain(base_dir):
            print("Custom ground mesh active from /workspace/uploads/terrain")

        # Step 1b.5: Add friction surface to the ground collision (always).
        patch_ground_friction(base_dir)

        # Step 1c: No imported asset -> recolor Forest3D's flat gray to
        # soil-brown. If textures were seeded its PBR material already stands.
        if not seeded:
            patch_ground_soil_colour(base_dir)

        # Step 2: Place crop + weed models. Weeds go via -d, NOT the yaml:
        # Forest3D's Density schema has no 'weed' field, so yaml weed counts
        # are dropped and its default (50) used; -d accepts arbitrary keys.
        # Forest3D is used for the ground mesh only: its row placement can
        # only draw straight X-aligned rows. Ask for a single crop and no
        # weeds; place_plants_along_rows() below removes it and writes the
        # real plants along the topo row paths.
        gen_cmd = [*base_cmd, 'generate', '-t', 'crop_rows',
                   '-b', str(base_dir),
                   '-d', json.dumps({'crop': 1, 'weed': 0})]
        if args.world_out:
            gen_cmd += ['-o', args.world_out]
        print(f"Running: {' '.join(gen_cmd)}")
        subprocess.run(gen_cmd, check=True)

        # Step 3: Track drive needs DART (see patch_world_physics_engine).
        if args.world_out:
            patch_world_physics_engine(args.world_out)

        # Step 3.5: Georeference for gz-sim-navsat-system (see patch_world_...).
        if args.world_out:
            patch_world_spherical_coordinates(args.world_out, gps_lat, gps_lon)

        # Step 4: Shift ALL models so the world aligns with the topo frame.
        # Terrain and crops are siblings at absolute world positions; moving
        # only the ground leaves crops stranded at origin.
        if args.world_out:
            patch_world_model_poses(args.world_out, terrain_offset)

        # Step 4.5: Write the plants along the topo row paths, in topo (world)
        # coordinates. After the pose shift above (which moves every include)
        # and before the terrain snap below.
        if args.world_out:
            place_plants_along_rows(
                args.world_out, rows, args.plant_spacing,
                args.crop_model or 'plant',
                args.weed_density if args.weed_density is not None else 10,
                models_path=args.models_path,
                weed_radius=max(args.row_width, 0.3))

        # Step 4.6: Snap every plant onto the terrain surface (custom mesh).
        if args.world_out:
            snap_plants_to_terrain(
                args.world_out,
                Path(args.models_path) / 'ground' / 'mesh' / 'terrain.stl')

        # Step 6: replace model://crop/ URIs to point at the user-selected model.
        if args.world_out and args.crop_model:
            patch_crop_model_uri(args.world_out, args.crop_model)

        # Step 7: Gazebo Sim ignores <include><scale> (gz-sim#195), so scale
        # the mesh <scale> in model.sdf directly. Always runs — even at
        # plant_scale=1.0 — so returning to 100% resets the mesh to its base
        # scale instead of leaving a stale factor patched in by an earlier run.
        if args.models_path:
            _categories_to_scale = (
                ['crop', 'weed'] if args.scale_category == 'all'
                else [args.scale_category]
            )
            _cat_model_map = {
                'crop': args.crop_model or 'plant',
                'weed': 'weed1',
                'irrigation': 'irrigation',
            }
            for _cat in _categories_to_scale:
                _model = _cat_model_map.get(_cat)
                if _model:
                    patch_model_mesh_scale(
                        args.models_path, _cat, _model, args.plant_scale)

        print("Done.")
