#!/usr/bin/env python3
"""
test_contour_planning.py
─────────────────────────
Runs the same recon.csv -> elevation grid -> reference contour -> contour
swaths pipeline as ui_node.py's _plan_contour_rows(), directly from the
command line — no NiceGUI, no ROS graph, no drawing a boundary by hand.

Run this INSIDE the devkit_ros container (needs fields2cover, shapely,
scipy, scikit-image — same environment ui_node normally runs in):

    source /opt/ros/jazzy/setup.bash
    source /workspace/install/setup.bash
    python3 test_contour_planning.py /workspace/maps/recon_logs/recon.csv

Field boundary
──────────────
_run_contour_f2c() needs a field boundary (corners_ll) to know where each
offset row should be clipped — that's not optional, see the "do we even
need to graphically select map area" discussion. Since there's no drawn
boundary here, this script builds one from the recon points' own bounding
box (+ a small pad), which is the "derive it from the CSV" shortcut
that's fine for a test/synthetic run, but is exactly the approximation
that's worse than a real drawn boundary in the field: a recon drive
usually turns before reaching the true edge, so a bbox-from-points
boundary will generally sit inside the real one.

Anchor consistency (the bug flagged in chat — now fixed)
──────────────────────────────────────────────────────────
field_centroid_xy(corners_ll) projects the boundary into local xy anchored
at corners_ll[0]. build_elevation_grid()'s origin_xy used to be in
whatever local xy frame recon.csv's own x,y columns were already in (i.e.
/odom's frame when it was logged) — a DIFFERENT frame than corners_ll[0]
in general, unless corners_ll[0] happened to be the same point /odom
zeroed out at. This bit for real with the France field-27 data (a ~372m
field): _plan_contour_rows() (ui_node.py) and recon_csv_to_obstacle_rings()
(dem.py) now re-anchor recon points onto corners_ll[0]/anchor_lat,lon via
their own lat/lon columns — load_recon_points() returns latlon for exactly
this — before building the elevation grid, so origin_xy and centroid_xy
share one frame regardless of where /odom happened to zero out or where
the boundary was drawn.

This script still builds its test boundary from the recon points' own
bounding box (+ pad) rather than a real drawn boundary, which is a
separate, unrelated approximation (see "Field boundary" above) — that
part is still worth keeping in mind for a real field.
"""

import argparse
import sys
from pathlib import Path

import numpy as np

# Make the repo's ROS packages importable without a full colcon build/source,
# if this is run straight from a checkout instead of inside the container.
_REPO_SRC = Path(__file__).resolve().parent / 'src'
for pkg in ('devkit_f2c_planner', 'devkit_ui'):
    candidate = _REPO_SRC / pkg
    if candidate.is_dir():
        sys.path.insert(0, str(candidate))

from devkit_f2c_planner.f2c_planner import (  # noqa: E402
    _f2c_latlon_to_xy,
    _f2c_xy_to_latlon,
    _run_contour_f2c,
    field_centroid_xy,
)
from devkit_ui.dem import (  # noqa: E402
    build_elevation_grid,
    load_recon_points,
    select_reference_contour_latlon,
)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('recon_csv', help='Path to recon.csv (stamp,lat,lon,alt,x,y)')
    ap.add_argument('--anchor-lat', type=float, default=48.0046000,
                     help='lat that recon.csv\'s x=0,y=0 corresponds to '
                          '(default: the sim GPS datum used elsewhere in this repo)')
    ap.add_argument('--anchor-lon', type=float, default=3.6644000)
    ap.add_argument('--boundary-pad-m', type=float, default=0.5,
                     help='pad the recon points\' bbox outward by this much '
                          'to build a test field boundary (default 0.5m)')
    ap.add_argument('--width', type=float, default=1.2, help='tool width, metres')
    ap.add_argument('--pad-m', type=float, default=0.0, help='obstacle padding, metres')
    ap.add_argument('--headland-m', type=float, default=0.0)
    ap.add_argument('--dem-resolution-m', type=float, default=1.0)
    ap.add_argument('--no-snake', action='store_true')
    args = ap.parse_args()

    xy_native, elevation, latlon = load_recon_points(args.recon_csv)
    print(f'[test] loaded {len(xy_native)} recon points from {args.recon_csv}')
    print(f'[test] elevation range: {elevation.min():.3f}m .. {elevation.max():.3f}m')

    x_min, y_min = xy_native[:, 0].min(), xy_native[:, 1].min()
    x_max, y_max = xy_native[:, 0].max(), xy_native[:, 1].max()
    pad = args.boundary_pad_m
    # Boundary corners in the recon points' OWN native frame, then
    # reprojected to lat/lon anchored at (x=0,y=0) -> (--anchor-lat/-lon).
    # This just needs SOME lat/lon anchor for the native frame — it no
    # longer needs to coincide with corners_ll[0], since the re-anchoring
    # step below (the actual fix) handles that regardless of what anchor
    # is picked here.
    corners_xy = [
        (x_min - pad, y_min - pad),
        (x_max + pad, y_min - pad),
        (x_max + pad, y_max + pad),
        (x_min - pad, y_max + pad),
    ]
    corners_ll = [_f2c_xy_to_latlon(x, y, args.anchor_lat, args.anchor_lon)
                  for x, y in corners_xy]
    print(f'[test] test boundary (bbox + {pad}m pad): '
          f'x=[{x_min - pad:.1f},{x_max + pad:.1f}] '
          f'y=[{y_min - pad:.1f},{y_max + pad:.1f}]')

    # The actual fix: re-anchor recon points onto corners_ll[0] via their
    # own lat/lon, so origin_xy ends up in the same frame field_centroid_xy()
    # computes centroid_xy in below — see the module docstring above.
    lat0, lon0 = corners_ll[0]
    xy = np.array([_f2c_latlon_to_xy(lat, lon, lat0, lon0) for lat, lon in latlon])
    elevation_grid, origin_xy, smoothing_used = build_elevation_grid(
        xy, elevation, args.dem_resolution_m)
    print(f'[test] elevation grid: {elevation_grid.shape}, '
          f'origin_xy={origin_xy}, smoothing={smoothing_used}')

    centroid_xy = field_centroid_xy(corners_ll)
    print(f'[test] field centroid (boundary frame): {centroid_xy}')

    reference_line_ll = select_reference_contour_latlon(
        elevation_grid, args.dem_resolution_m, origin_xy, centroid_xy, lat0, lon0)

    if reference_line_ll is None:
        print('[test] select_reference_contour_latlon returned None — field '
              'too flat, or centroid landed outside the elevation grid '
              '(the anchor-mismatch failure mode described above). '
              'do_plan() would fall back to straight _run_f2c() rows here.')
        return 1

    print(f'[test] reference contour: {len(reference_line_ll)} points')

    swaths = _run_contour_f2c(
        corners_ll, [], reference_line_ll, args.width,
        args.pad_m, args.headland_m, not args.no_snake)

    print(f'[test] {len(swaths)} contour rows generated')
    for i, row in enumerate(swaths):
        print(f'  row {i}: {len(row)} pts, '
              f'{row[0]} -> {row[-1]}')

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
