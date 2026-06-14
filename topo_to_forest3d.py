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
        furrow_width = 0.1

    # Clamp to Forest3D's crop_rows schema minimums so an unusual map can never
    # emit a config the generator rejects (row_width >= 0.2, furrow >= 0.1).
    row_width = max(row_width, 0.2)
    furrow_width = max(furrow_width, 0.1)

    # Field extent: bounding box of all nodes + headland margin. Length runs
    # along the rows (row_axis); width runs across them (cross_axis).
    along = [r['a'][row_axis] for r in rows] + [r['b'][row_axis] for r in rows]
    across = [r['a'][cross_axis] for r in rows] + [r['b'][cross_axis] for r in rows]
    row_length = max(along) - min(along)
    field_length = row_length + 1 * headland_width
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
    }, gps_lat, gps_lon, plants_per_row, derived_density


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
    ap.add_argument('--row-width', type=float, default=0.8,
                    help='Default crop row width (m) if spacing cannot be '
                         'measured from the data')
    ap.add_argument('--plant-spacing', type=float, default=0.8,
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
    args = ap.parse_args()

    nodes = load_topo(args.topo)
    rows = extract_rows(nodes)
    if not rows:
        sys.exit("ERROR: no complete rows found in topo map")

    print(f"Loaded {len(rows)} rows from {args.topo}")

    params, gps_lat, gps_lon, plants_per_row, derived_density = compute_field_params(
        rows, args.headland, args.row_width, args.plant_spacing)

    # density is only a global ceiling in Forest3D's placement loop. Default it
    # to the geometry-derived count so it never truncates the rows; honour an
    # explicit --density override when given.
    density = args.density if args.density is not None else derived_density
    print(f"  Plants/row: {plants_per_row}  ->  density cap: {density}")

    write_forest3d_yaml(params, gps_lat, gps_lon, args.out, density,
                        args.models_path)

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
        print("Done.")
