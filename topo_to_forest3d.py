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
import subprocess
import sys
from pathlib import Path
from typing import Optional

try:
    import yaml
except ImportError:
    yaml = None


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
            'x': float(pos['x']),
            'y': float(pos['y']),
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

    Uses the FIRST node in the saved map — i.e. whichever node the user
    placed first in the devkit_ui cockpit — rather than any specific name,
    since hand-authored UI maps don't follow get_maize_topo.py's HL_S/HL_W/
    HOME naming convention at all.

    One exception: a node literally named HOME is skipped. get_maize_topo.py
    (the bootstrap path that runs when no map has been authored yet) writes
    HOME first in the file, hardcoded to (0.0, 0.0) regardless of where the
    field actually is — every other node it writes (HL_S/HL_W, R{i}_IN/OUT)
    is derived from the real crop GT data, but HOME isn't. Before
    terrain_offset existed this didn't matter (world was never shifted, so
    world (0,0) and HOME (0,0) coincided by luck); once the world is shifted
    by terrain_offset — derived from that same row/headland bounding box —
    adding it to HOME's unrelated fixed origin lands off the edge of the
    generated terrain instead of on it. So: take the first node, unless it's
    HOME, in which case take the next one (HL_S/HL_W for bootstrap maps,
    still field-derived).

    Falls back to the first row's entry/IN node if every node is HOME or the
    map is otherwise unreachable this way (shouldn't normally happen).
    """
    for name, nd in nodes.items():
        if name == 'HOME':
            continue
        return nd['x'], nd['y']
    if rows:
        return rows[0]['a']
    return 0.0, 0.0


def extract_rows(nodes):
    by_rid = {}
    for name, nd in nodes.items():
        rid = nd['row_id']
        if rid is None:
            continue
        by_rid.setdefault(rid, {})[nd['row_role'] or name] = nd

    rows = []
    for rid, ends in sorted(by_rid.items()):
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
        gps_lat = in_nd.get('gps_lat') or out_nd.get('gps_lat')
        gps_lon = in_nd.get('gps_lon') or out_nd.get('gps_lon')
        rows.append({
            'rid': rid,
            'a': (in_nd['x'], in_nd['y']),
            'b': (out_nd['x'], out_nd['y']),
            'gps_lat': gps_lat,
            'gps_lon': gps_lon,
        })
    return rows


def compute_field_params(rows, headland_width, default_row_width, plant_spacing):
    if len(rows) < 1:
        sys.exit("ERROR: need at least 1 row")

    # Determine row direction: the axis each IN→OUT segment actually extends
    # along (its mean magnitude). Using the *spread* of the direction vectors
    # fails when all rows are parallel — every spread collapses to ~0 and the
    # tie silently picks the wrong axis.
    mean_dx = sum(abs(r['b'][0] - r['a'][0]) for r in rows) / len(rows)
    mean_dy = sum(abs(r['b'][1] - r['a'][1]) for r in rows) / len(rows)
    if mean_dx >= mean_dy:
        row_axis, cross_axis = 0, 1
    else:
        row_axis, cross_axis = 1, 0

    # Row centres along the cross-axis.
    centres = [(r['a'][cross_axis] + r['b'][cross_axis]) / 2 for r in rows]
    centres.sort()

    num_rows = len(rows)

    # Average spacing between adjacent row centres.
    spacing = 0.0
    if num_rows > 1:
        gaps = [centres[i+1] - centres[i] for i in range(num_rows - 1)]
        spacing = sum(gaps) / len(gaps)

    row_width = default_row_width
    if spacing > 0:
        furrow_width = spacing - row_width
        if furrow_width < 0.1:
            furrow_width = spacing * 0.4
            row_width = spacing - furrow_width
    else:
        # Single row (or no measurable spacing): fall back to sane defaults.
        furrow_width = 0.3

    # Clamp to Forest3D's crop_rows schema minimums so an unusual map can never
    # emit a config the generator rejects (row_width >= 0.2, furrow >= 0.1).
    row_width = max(row_width, 0.2)
    furrow_width = max(furrow_width, 0.1)

    # Field extent: bounding box of all nodes + headland margin. Length runs
    # along the rows (row_axis); width runs across them (cross_axis).
    along = [r['a'][row_axis] for r in rows] + [r['b'][row_axis] for r in rows]
    across = [r['a'][cross_axis] for r in rows] + [r['b'][cross_axis] for r in rows]
    row_length = max(along) - min(along)
    field_length = row_length + 2 * headland_width
    field_width = max(across) - min(across) + 2 * headland_width

    # Plants per row mirrors Forest3D's RowPlacement.place():
    #   n_plants = max(1, int(row_length / plant_spacing))
    # where row_length is the planted span between headlands (the bbox extent
    # along the rows, before the headland margin is added on each end).
    plants_per_row = max(1, int(row_length / plant_spacing))
    derived_density = num_rows * plants_per_row

    # GPS origin (use first row with GPS data, or None).
    gps_lat = next((r['gps_lat'] for r in rows if r['gps_lat'] is not None), None)
    gps_lon = next((r['gps_lon'] for r in rows if r['gps_lon'] is not None), None)

    # Resolution must be fine enough that CropRowTerrain's half_row_idx >= 1
    # (i.e. the row profile covers at least one cell on each side of centre).
    # With half_row = row_width/2, we need resolution <= row_width/2.
    # Use row_width/2.5 for a coarser mesh (fewer triangles → faster Gazebo).
    resolution = max(0.1, min(0.25, row_width / 2.5))

    # Terrain pose offset: shift the Forest3D world (terrain + all crop models)
    # so that Forest3D's raised-bed row centres land on the topo node positions.
    #
    # Forest3D places row i (0-indexed) at local cross position:
    #   y_local_i = -field_width/2 + headland + row_width/2 + i*spacing
    # The centroid of all N local row positions is:
    #   local_centroid = -field_width/2 + headland + row_width/2 + (N-1)*spacing/2
    # Since field_width = cross_span + 2*headland and (N-1)*spacing ≈ cross_span:
    #   local_centroid ≈ row_width/2    (NOT zero)
    # So the required offset is: topo_cross_centre - row_width/2.
    topo_along_centre = (max(along) + min(along)) / 2.0
    topo_cross_centre = (max(across) + min(across)) / 2.0
    terrain_offset_cross = round(topo_cross_centre - row_width / 2.0, 4)
    terrain_offset_along = round(topo_along_centre, 4)
    if row_axis == 0:    # rows run along X, cross is Y
        topo_x_centre, topo_y_centre = terrain_offset_along, terrain_offset_cross
    else:                # rows run along Y, cross is X
        topo_x_centre, topo_y_centre = terrain_offset_cross, terrain_offset_along

    return {
        'field_length': round(max(field_length, 5.0), 2),
        'field_width': round(max(field_width, 5.0), 2),
        'num_rows': num_rows,
        'row_width': round(row_width, 3),
        'furrow_width': round(furrow_width, 3),
        'headland_width': headland_width,
        'row_height': 0.15,
        'row_profile': 'rounded',
        'plant_spacing': plant_spacing,
        'stagger': 0.0,
        'resolution': resolution,
    }, gps_lat, gps_lon, plants_per_row, derived_density, (
        round(topo_x_centre, 4),
        round(topo_y_centre, 4),
    )


# Soil assets imported from the NiceGUI cockpit land here (a host dir mounted
# at /workspace/uploads, persisted across image rebuilds). Only the image maps
# are kept; Forest3D classifies them by filename when it builds the ground SDF.
SOIL_UPLOAD_DIR = Path('/workspace/uploads/soil_custom/textures')
_IMG_EXTS = ('.jpg', '.jpeg', '.png')

# Forest3D emits this flat gray material on the ground when no texture is found.
GRAY_MATERIAL = (
    '<ambient>0.6 0.6 0.6 1</ambient>\n'
    '                    <diffuse>0.8 0.8 0.8 1</diffuse>'
)
# Fallback when no soil asset has been imported: a plain soil-brown colour (no
# texture files needed) so the ground doesn't look like concrete.
FLAT_SOIL_MATERIAL = (
    '<ambient>0.25 0.15 0.07 1</ambient>\n'
    '                    <diffuse>0.40 0.25 0.12 1</diffuse>\n'
    '                    <specular>0.02 0.02 0.02 1</specular>'
)


def seed_ground_textures(base_dir):
    """Copy imported soil maps into the ground model's texture dir.

    Forest3D's process_terrain() scans <ground>/texture/ via _find_textures()
    and, when images are present, emits a PBR <albedo_map>/<normal_map>/
    <roughness_map> material referencing model://ground/texture/<file> — exactly
    the behaviour we want, no Blender required. So we just stage the imported
    images there *before* `forest3d terrain crop_rows` runs.

    Returns the number of image files staged (0 if nothing was imported).
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


# Forest3D's generated ground <collision> has no <surface><friction> block at
# all. gz-sim-tracked-vehicle-system / TrackController publish/consume valid
# non-zero track_cmd_vel commands (confirmed via `gz topic -e`) and DART
# physics is active (see patch_world_physics_engine below), but with zero
# friction defined at the track/ground contact the belts have nothing to grip
# — /odom stays at the noise floor with no error. Inserted right after the
# </geometry> close of the ground collision block, the one fixed string every
# Forest3D-generated ground model.sdf is expected to contain.
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


# Forest3D's world template defaults to the ODE physics engine. The Sowbot
# model's track drive (gz-sim-tracked-vehicle-system / TrackController) only
# transmits motion through DART's contact-surface-velocity API — under ODE,
# TrackedVehicle and TrackController publish/consume valid non-zero commands
# (confirmed via `gz topic -e`) but the physics step silently never moves the
# robot, with no error. Swap the engine to DART so the spawned robot can
# actually drive.
ODE_PHYSICS = "<physics type='ode'>"
DART_PHYSICS = "<physics type='dart'>"


def patch_world_physics_engine(world_path):
    """Switch the generated world's physics engine from ODE to DART.

    Forest3D regenerates the world file from scratch on every run (manage.py
    does `rm -f {topo_world}` before each `forest3d generate`), so this must
    run every time, not just once by hand. Warns (rather than silently
    no-op'ing) if the expected ODE tag isn't found, so a Forest3D template
    change becomes visible instead of leaving an unpatched, non-functional
    world.
    """
    world_path = Path(world_path)
    if not world_path.exists():
        print(f"WARNING: {world_path} not found — skipping physics engine patch",
              file=sys.stderr)
        return
    world_text = world_path.read_text()
    if DART_PHYSICS in world_text:
        print(f"{world_path} already uses DART physics — nothing to patch")
        return
    if ODE_PHYSICS not in world_text:
        print(f"WARNING: expected '{ODE_PHYSICS}' not found in {world_path}; "
              "Forest3D may have changed its physics default — leaving as-is. "
              "TrackedVehicle/TrackController require DART; verify manually.",
              file=sys.stderr)
        return
    world_path.write_text(world_text.replace(ODE_PHYSICS, DART_PHYSICS))
    print(f"Patched {world_path}: physics engine ode -> dart "
          "(required for TrackedVehicle/TrackController motion)")



def patch_world_model_poses(world_path, terrain_offset):
    """Shift ALL spawned models by terrain_offset so the Forest3D world aligns
    with the topo nav coordinate frame.

    Forest3D generates terrain centred at (0,0) and places every crop model at
    an absolute world position relative to that origin.  The terrain and crop
    models are siblings in the Entity Tree — they are NOT parent/child — so
    moving only the ground leaves crops stranded at origin.  This function
    adds terrain_offset to the <pose> of every <include> block so terrain and
    crops move as one rigid group.

    terrain_offset is (x, y): corrected topo centroid minus the row_width/2
    local-frame bias (see compute_field_params).
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


def write_spawn_pose(x, y, z, output_path):
    """Write the robot spawn pose so sim.launch.py can read it at startup.

    sim.launch.py previously hardcoded -x 0.0 -y 0.0, which only matched
    HOME's world position by coincidence (before terrain_offset existed,
    nothing in the world ever moved). Writing it here keeps spawn position
    derived from the same topo+offset math that places the terrain, instead
    of drifting out of sync with it.
    """
    Path(output_path).write_text(f"{x} {y} {z}\n")
    print(f"Wrote spawn pose ({x}, {y}, {z}) to {output_path}")


def write_forest3d_yaml(params, gps_lat, gps_lon, output_path, density_crop, model_path):
    config = {
        'terrain': {
            'type': 'crop_rows',
            'scale_factor': 1.0,
            'material_name': 'Terrain/Soil',
            'crop_rows': params,
        },
        'density': {
            'crop': density_crop,
        },
        'paths': {},
    }
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
    ap.add_argument('--plant-spacing', type=float, default=1.2,
                    help='Spacing between plants along a row (m). Forest3D '
                         'places int(row_length / plant_spacing) plants per row.')
    ap.add_argument('--density', type=int, default=None,
                    help='Global cap on crop models to place. Defaults to '
                         'num_rows x plants_per_row derived from the map so the '
                         'cap never clips the geometry; pass a value to override.')
    ap.add_argument('--models-path', type=str, default=None,
                    help='Path to Forest3D models directory')
    ap.add_argument('--generate', action='store_true',
                    help='Also run forest3d generate after writing config')
    ap.add_argument('--world-out', default=None,
                    help='Output world path for forest3d generate (optional)')
    ap.add_argument('--spawn-out', default='/workspace/spawn_pose.txt',
                    help='Output path for the robot spawn pose (x y z), '
                         'read by sim.launch.py instead of a hardcoded origin')
    ap.add_argument('--spawn-z', type=float, default=0.01,
                    help='Spawn height above ground (base_footprint), '
                         'matches sim.launch.py default')
    args = ap.parse_args()

    nodes = load_topo(args.topo)
    rows = extract_rows(nodes)
    if not rows:
        sys.exit("ERROR: no complete rows found in topo map")

    print(f"Loaded {len(rows)} rows from {args.topo}")

    params, gps_lat, gps_lon, plants_per_row, derived_density, terrain_offset = compute_field_params(
        rows, args.headland, args.row_width, args.plant_spacing)

    # Robot spawn point: the topo map's HOME node (or first row's IN node as
    # fallback), shifted by the same terrain_offset that's about to be baked
    # into the world. Without this, the robot spawns at literal world (0,0)
    # (sim.launch.py's hardcoded default) while the terrain — and HOME's
    # *intended* position relative to it — has moved by terrain_offset.
    # Before terrain_offset existed this was harmless because nothing in the
    # world ever moved; once the world is shifted, HOME and the hardcoded
    # spawn silently drift apart.
    spawn_topo_x, spawn_topo_y = find_spawn_node(nodes, rows)
    spawn_world_x = round(spawn_topo_x + terrain_offset[0], 4)
    spawn_world_y = round(spawn_topo_y + terrain_offset[1], 4)
    print(f"  Spawn point (topo -> world): "
          f"({spawn_topo_x}, {spawn_topo_y}) + offset {terrain_offset} "
          f"= ({spawn_world_x}, {spawn_world_y})")

    # density is only a global ceiling in Forest3D's placement loop. Default it
    # to the geometry-derived count so it never truncates the rows; honour an
    # explicit --density override when given.
    density = args.density if args.density is not None else derived_density
    print(f"  Plants/row: {plants_per_row}  ->  density cap: {density}")

    write_forest3d_yaml(params, gps_lat, gps_lon, args.out, density,
                        args.models_path)
    write_spawn_pose(spawn_world_x, spawn_world_y, args.spawn_z, args.spawn_out)

    if args.generate:
        base_cmd = ['python3', '-m', 'forest3d']
        base_dir = Path(args.out).parent

        # Step 1a: Stage any imported soil maps into the ground texture dir
        # BEFORE generating, so Forest3D picks them up and writes a PBR
        # material itself (model://ground/texture/...).
        seeded = seed_ground_textures(base_dir)
        if seeded:
            print(f"Staged {seeded} soil texture map(s) for the ground model")

        # Step 1b: Generate the crop-row terrain mesh (models/ground/).
        terrain_cmd = [*base_cmd, 'terrain', 'crop_rows']
        print(f"Running: {' '.join(terrain_cmd)}")
        subprocess.run(terrain_cmd, check=True)

        # Step 1b.5: Forest3D's ground collision has no friction surface at
        # all (see patch_ground_friction docstring) — must run every time,
        # independent of whether a soil texture was seeded.
        patch_ground_friction(base_dir)

        # Step 1c: With no imported asset, Forest3D leaves a flat gray material —
        # recolour it to soil-brown. When textures were seeded it already wrote a
        # PBR material, so leave that untouched.
        if not seeded:
            patch_ground_soil_colour(base_dir)

        # Step 2: Place crop models on the terrain.
        gen_cmd = [*base_cmd, 'generate', '-t', 'crop_rows',
                   '-b', str(base_dir)]
        if args.world_out:
            gen_cmd += ['-o', args.world_out]
        print(f"Running: {' '.join(gen_cmd)}")
        subprocess.run(gen_cmd, check=True)

        # Step 3: Forest3D's world template defaults to ODE; the Sowbot track
        # drive needs DART (see patch_world_physics_engine docstring).
        if args.world_out:
            patch_world_physics_engine(args.world_out)

        # Step 4: shift ALL models (terrain + every crop_N) so the Forest3D
        # world aligns with the topo nav coordinate frame.  Terrain and crops
        # are siblings at absolute world positions; only shifting the ground
        # (old approach) leaves crops stranded at origin.
        if args.world_out:
            patch_world_model_poses(args.world_out, terrain_offset)

        print("Done.")
