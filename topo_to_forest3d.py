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
        # Track which node's (x, y) the GPS fix actually belongs to — needed
        # to later back-solve the lat/lon of topo-frame (0, 0), since a GPS
        # tag is only meaningful together with the local coordinate it was
        # recorded at (see compute_field_params' gps origin back-solve).
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
            'gps_lat': gps_lat,
            'gps_lon': gps_lon,
            'gps_x': gps_x,
            'gps_y': gps_y,
        })
    return rows


def compute_field_params(rows, headland_width, default_row_width, plant_spacing,
                          min_furrow_width=0.1):
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

    # min_furrow_width is the threshold that decides whether the 40%-of-
    # spacing auto-correction kicks in below. Previously this was hardcoded
    # to 0.1, conflating two different things: Forest3D's hard schema
    # minimum (furrow_width >= 0.1, fixed, never changes — see the final
    # clamp further down) vs. "how narrow a furrow is the caller actually
    # willing to accept before we override their row_width choice". With
    # only one value, there was no way to ask for "furrows as narrow as
    # Forest3D allows" — pushing row_width up close to spacing to shrink the
    # furrow just re-triggered the same 0.1 check and got silently widened
    # back out to spacing*0.4, overriding the caller's intent every time.
    row_width = default_row_width
    if spacing > 0:
        furrow_width = spacing - row_width
        if furrow_width < min_furrow_width:
            furrow_width = spacing * 0.4
            row_width = spacing - furrow_width
    else:
        # Single row (or no measurable spacing): fall back to sane defaults.
        furrow_width = max(min_furrow_width, 0.1)

    # Clamp to Forest3D's crop_rows schema minimums so an unusual map can never
    # emit a config the generator rejects (row_width >= 0.2, furrow >= 0.1).
    # This 0.1 is Forest3D's actual schema floor (Forest3DConfig's pydantic
    # validator requires furrow_width >= 0.1) — distinct from min_furrow_width
    # above, which is just the caller's preference for when to trust their
    # own row_width vs. fall back to auto-correction. The schema floor always
    # applies regardless of what the caller asked for, since going below it
    # makes `forest3d generate` raise ValidationError and abort entirely.
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

    # GPS origin: the lat/lon of topo-frame local (0, 0) — NOT the raw GPS
    # tag off whichever row happened to have one first.
    #
    # A GPS tag on a topo node (e.g. F2C_R1_IN) is the fix recorded AT THAT
    # NODE's (x, y), which is generally nowhere near (0, 0) — topo node
    # coordinates are already in the world/map frame (see find_spawn_node's
    # docstring), so (0, 0) is a specific point out in the field, not
    # "wherever the first GPS-tagged node happens to be".
    #
    # gz-sim-navsat-system's <spherical_coordinates> block declares
    # "local (0,0,0) = this lat/lon" — so what it needs is the geodetic
    # position of the ORIGIN, back-solved from a known (lat, lon) <-> (x, y)
    # pair using a flat-earth approximation (fine at field scale, <1km):
    #   1 deg latitude  ~= 111320 m
    #   1 deg longitude ~= 111320 * cos(latitude) m
    # The known point sits at local (gps_x, gps_y) relative to the origin,
    # so the origin sits at local (-gps_x, -gps_y) relative to the known
    # point — convert that offset to degrees and add it to the known fix.
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
        'row_height': 0.0015,
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


# Forest3D's generated world uses type="ignored" (double-quoted), which tells
# Gazebo to use its compiled-in default (ODE). Hand-authored worlds in the repo
# use type='ode' (single-quoted). Both must be replaced with type="dart".
# Previously only the single-quoted ode variant was matched, so the patch
# silently no-op'd on every Forest3D-generated world, leaving ODE active and
# TrackedVehicle/TrackController without SetContactPropertiesCallbackFeature
# support — the robot received commands but physics never moved it.
_PHYSICS_TYPE_RE = re.compile(r'(<physics\b[^>]*\btype=)(["\'])[^"\']*\2')
DART_TYPE = "dartsim"


def patch_world_physics_engine(world_path):
    """Switch the generated world's physics engine to DART.

    Matches any <physics ...> tag's type= attribute via regex, regardless
    of attribute order/quoting, since Forest3D regenerates the world file
    from scratch on every run and its template has changed the exact
    attribute layout before. Warns if no <physics> tag is found at all so
    template changes are visible.
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
    """Insert a <spherical_coordinates> block so gz-sim-navsat-system has a
    real WGS84 datum to report fixes against.

    Forest3D's world template has no <spherical_coordinates> tag at all.
    Without one, gz-sim-navsat-system falls back to its hardcoded default
    origin (lat=0, lon=0) — every fix it publishes is then a frozen,
    degenerate point at (0,0) regardless of where the robot actually is in
    the world, which is exactly the constant-distance-from-reference
    rejection fusioncore's outlier gate has been logging. Forest3D
    regenerates the world from scratch on every run (same reason
    patch_world_physics_engine must run every time), so this has to run
    every time too, not just once.

    gps_lat/gps_lon must be the geodetic position of topo-frame local
    (0, 0) — i.e. already back-solved by compute_field_params from a
    topo node's own (lat, lon, x, y), NOT a raw node GPS tag passed
    through unchanged. A node's own fix corresponds to that node's (x, y),
    which is generally nowhere near (0, 0); declaring it as the origin
    datum directly shifts the whole georeference by that node's offset,
    which previously showed up as the robot appearing to spawn in the
    wrong place relative to the map once anything (fusioncore's UKF,
    robot_localization, etc.) converted a GPS fix back to local ENU using
    this datum.
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


def cull_crops_outside_field(world_path, rows, tol):
    """Delete crop models outside their row's authored IN->OUT span.

    Forest3D plants every row the full rectangular field_length (no per-row
    length support), so on irregular real fields it fabricates crops past the
    boundary. Runs after patch_world_model_poses (crops already in topo frame):
    snap each crop to nearest row by cross-axis, drop it if its along-axis coord
    is outside [min(IN,OUT), max(IN,OUT)] +/- tol. Trims plants only; the
    terrain mesh stays a full rectangle. Crop models = uri model://crop/...
    """
    import xml.etree.ElementTree as _ET

    world_path = Path(world_path)
    if not world_path.exists():
        print(f"WARNING: {world_path} not found — skipping crop cull",
              file=sys.stderr)
        return

    mean_dx = sum(abs(r['b'][0] - r['a'][0]) for r in rows) / len(rows)
    mean_dy = sum(abs(r['b'][1] - r['a'][1]) for r in rows) / len(rows)
    row_axis, cross_axis = (0, 1) if mean_dx >= mean_dy else (1, 0)

    bands = [((r['a'][cross_axis] + r['b'][cross_axis]) / 2.0,
              min(r['a'][row_axis], r['b'][row_axis]),
              max(r['a'][row_axis], r['b'][row_axis])) for r in rows]

    try:
        root = _ET.parse(world_path).getroot()
    except _ET.ParseError as exc:
        print(f"WARNING: cannot parse {world_path} — skipping cull: {exc}",
              file=sys.stderr)
        return

    world_elem = root.find("world") or root
    crops = removed = 0
    for include in list(world_elem.findall("include")):
        if not (include.findtext("uri") or "").startswith("model://crop/"):
            continue
        crops += 1
        parts = (include.findtext("pose") or "").split()
        if len(parts) < 2:
            continue
        pos = (float(parts[0]), float(parts[1]))
        along, cross = pos[row_axis], pos[cross_axis]
        _, lo, hi = min(bands, key=lambda b: abs(b[0] - cross))
        if along < lo - tol or along > hi + tol:
            world_elem.remove(include)
            removed += 1

    if crops == 0:
        print(f"WARNING: no model://crop/ includes in {world_path} — nothing "
              "culled; verify crop category. ", file=sys.stderr)
        return

    world_path.write_text('<?xml version="1.0" ?>\n'
                          + _ET.tostring(root, encoding="unicode"))
    print(f"Culled {removed}/{crops} crop(s) outside authored row spans "
          f"(tol={tol:.2f}m)")


def patch_crop_model_uri(world_path, model_name):
    """Replace model variant in model://crop/ URIs with the selected model.

    Forest3D generates model://crop/<variant> based on whatever subdirectory
    it randomly picks from models/crop/. This forces all crop includes to
    point at the user-selected model instead.
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
    """Scale the <mesh><scale> inside model.sdf so Gazebo renders it correctly.

    Gazebo Sim ignores <include><scale> in world files (gz-sim#195).  The only
    reliable way to resize a model is to patch the mesh <scale> in model.sdf
    itself.

    The *base* scale (the original design value, e.g. 0.7 for crop/plant) is
    persisted in a .base_scale file next to model.sdf so repeated calls compose
    correctly: new_scale = base_scale * user_factor.
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
    """Write the robot spawn pose so sim.launch.py can read it at startup.

    sim.launch.py previously hardcoded -x 0.0 -y 0.0, which only matched
    HOME's world position by coincidence (before terrain_offset existed,
    nothing in the world ever moved). Writing it here keeps spawn position
    derived from the same topo+offset math that places the terrain, instead
    of drifting out of sync with it.
    """
    Path(output_path).write_text(f"{x} {y} {z}\n")
    print(f"Wrote spawn pose ({x}, {y}, {z}) to {output_path}")


def write_forest3d_yaml(params, gps_lat, gps_lon, output_path, density_crop,
                        model_path, density_weed=0):
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
                         '40%%-of-spacing auto-correction kicks in. Defaults '
                         "to Forest3D's own schema floor (0.1) — set "
                         '--row-width close to your measured row spacing '
                         'and leave this at 0.1 to make furrows (the gaps '
                         'between raised beds) as narrow as Forest3D allows. '
                         'Forest3D itself never accepts less than 0.1 '
                         'regardless of this setting; passing 0.0 here just '
                         'means "accept Forest3D\'s own minimum", not '
                         '"disable furrows entirely" (furrows cannot be '
                         'fully removed — 0.1m is the generator\'s hard '
                         'floor).')
    ap.add_argument('--plant-spacing', type=float, default=1.2,
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
                    help='Global cap on crop models to place. Defaults to '
                         'num_rows x plants_per_row derived from the map so the '
                         'cap never clips the geometry; pass a value to override.')
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

    # Forest3D's hard schema floor is 0.1 — clamp the requested minimum up to
    # that so a caller who passes e.g. 0.0 (meaning "as narrow as possible")
    # gets the actual achievable floor, rather than something that would
    # still bounce off the inner clamp in compute_field_params for the wrong
    # reason.
    min_furrow_width = max(args.min_furrow_width, 0.1)

    params, gps_lat, gps_lon, plants_per_row, derived_density, terrain_offset = compute_field_params(
        rows, args.headland, args.row_width, args.plant_spacing,
        min_furrow_width=min_furrow_width)

    # Robot spawn point: the topo map's HOME node (or first row's IN node as
    # fallback), shifted by the same terrain_offset that's about to be baked
    # into the world. Without this, the robot spawns at literal world (0,0)
    # (sim.launch.py's hardcoded default) while the terrain — and HOME's
    # *intended* position relative to it — has moved by terrain_offset.
    # Before terrain_offset existed this was harmless because nothing in the
    # world ever moved; once the world is shifted, HOME and the hardcoded
    # spawn silently drift apart.
    spawn_topo_x, spawn_topo_y = find_spawn_node(nodes, rows)
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
    print(f"  Spawn point (topo node, used as-is — no terrain_offset applied): "
          f"({spawn_world_x}, {spawn_world_y})")

    # density is only a global ceiling in Forest3D's placement loop. Default it
    # to the geometry-derived count so it never truncates the rows; honour an
    # explicit --density override when given.
    density = args.density if args.density is not None else derived_density
    print(f"  Plants/row: {plants_per_row}  ->  density cap: {density}")

    # Weeds cluster near crops. Scale them against the crop count so a real
    # weedy field (several weeds per plant) is achievable: --weed-density is a
    # percentage (0-100) multiplied by WEEDS_PER_CROP. Default to ~65% of the
    # crop count for a visibly weedy field (above Forest3D's built-in default
    # of 50). Pass 0 to disable weeds.
    weed_density = (int(density * WEEDS_PER_CROP * (args.weed_density / 100))
                    if args.weed_density is not None
                    else int(density * 0.65 * WEEDS_PER_CROP))
    print(f"  Weed density: {weed_density} "
          f"({args.weed_density if args.weed_density is not None else 65}% of "
          f"{density} crops at {WEEDS_PER_CROP}x)")
    write_forest3d_yaml(params, gps_lat, gps_lon, args.out, density,
                        args.models_path, weed_density)
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

        # Step 2: Place crop + weed models. Density via CLI -d, NOT the yaml:
        # Forest3D's Density schema has no 'weed' field, so a weed count in
        # forest3d.yaml is dropped and its default (50) used. The -d path honours
        # arbitrary keys, so it's the only way to control weeds (and set weed=0).
        gen_cmd = [*base_cmd, 'generate', '-t', 'crop_rows',
                   '-b', str(base_dir),
                   '-d', json.dumps({'crop': density, 'weed': weed_density})]
        if args.world_out:
            gen_cmd += ['-o', args.world_out]
        print(f"Running: {' '.join(gen_cmd)}")
        subprocess.run(gen_cmd, check=True)

        # Step 3: Forest3D's world template defaults to ODE/ignored; the Sowbot
        # track drive needs DART (see patch_world_physics_engine docstring).
        if args.world_out:
            patch_world_physics_engine(args.world_out)

        # Step 3.5: Forest3D's world template has no georeference at all —
        # without it gz-sim-navsat-system reports every fix frozen at (0,0)
        # (see patch_world_spherical_coordinates docstring).
        if args.world_out:
            patch_world_spherical_coordinates(args.world_out, gps_lat, gps_lon)

        # Step 4: shift ALL models (terrain + every crop_N) so the Forest3D
        # world aligns with the topo nav coordinate frame.  Terrain and crops
        # are siblings at absolute world positions; only shifting the ground
        # (old approach) leaves crops stranded at origin.
        if args.world_out:
            patch_world_model_poses(args.world_out, terrain_offset)

        # Step 5: trim crops to each row's real IN->OUT span (irregular fields).
        if args.world_out:
            cull_crops_outside_field(args.world_out, rows, args.plant_spacing / 2)

        # Step 6: replace model://crop/ URIs to point at the user-selected model.
        if args.world_out and args.crop_model:
            patch_crop_model_uri(args.world_out, args.crop_model)

        # Step 7: apply uniform scale to selected category (or all).
        # Gazebo Sim ignores <include><scale> (gz-sim#195) — patch the
        # mesh <scale> in model.sdf directly so the change actually renders.
        if args.models_path and args.plant_scale != 1.0:
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
