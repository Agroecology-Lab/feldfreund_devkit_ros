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

Anchor consistency (the bug flagged in chat)
─────────────────────────────────────────────
field_centroid_xy(corners_ll) projects the boundary into local xy anchored
at corners_ll[0]. build_elevation_grid()'s origin_xy is in whatever local
xy frame recon.csv's own x,y columns are already in (i.e. /odom's frame
when it was logged). Those two are DIFFERENT frames unless corners_ll[0]
happens to be the same point as wherever /odom zeroed out.

This script sidesteps that by construction: it sets corners_ll[0] to the
CSV's own (x=0, y=0) point reprojected through the SAME lat/lon anchor
recon.csv was generated with (via --anchor-lat/--anchor-lon, defaulting to
the sim datum used elsewhere in this repo's fake data). That makes the
happy path work so you can test the contour math itself. It does NOT fix
the underlying bug — against a real recon drive, corners_ll[0] is wherever
you happened to click first on the web map, not wherever /odom's origin
was, and the mismatch will still bite. Fixing that properly means
reprojecting the boundary into the recon CSV's own frame (or vice versa)
before computing centroid_xy, not just picking a convenient test anchor.
"""

import argparse
import sys
from pathlib import Path

# Make the repo's ROS packages importable without a full colcon build/source,
# if this is run straight from a checkout instead of inside the container.
_REPO_SRC = Path(__file__).resolve().parent / 'src'
for pkg in ('devkit_f2c_planner', 'devkit_ui'):
    candidate = _REPO_SRC / pkg
    if candidate.is_dir():
        sys.path.insert(0, str(candidate))

from devkit_f2c_planner.f2c_planner import (  # noqa: E402
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
    ap.add_argument('--anchor-lat', type=float, default=9.0450717,
                     help='lat that recon.csv\'s x=0,y=0 corresponds to '
                          '(default: the sim GPS datum used elsewhere in this repo)')
    ap.add_argument('--anchor-lon', type=float, default=77.7920577)
    ap.add_argument('--boundary-pad-m', type=float, default=0.5,
                     help='pad the recon points\' bbox outward by this much '
                          'to build a test field boundary (default 0.5m)')
    ap.add_argument('--width', type=float, default=1.2, help='tool width, metres')
    ap.add_argument('--pad-m', type=float, default=0.0, help='obstacle padding, metres')
    ap.add_argument('--headland-m', type=float, default=0.0)
    ap.add_argument('--dem-resolution-m', type=float, default=1.0)
    ap.add_argument('--no-snake', action='store_true')
    args = ap.parse_args()

    xy, elevation = load_recon_points(args.recon_csv)
    print(f'[test] loaded {len(xy)} recon points from {args.recon_csv}')
    print(f'[test] elevation range: {elevation.min():.3f}m .. {elevation.max():.3f}m')

    x_min, y_min = xy[:, 0].min(), xy[:, 1].min()
    x_max, y_max = xy[:, 0].max(), xy[:, 1].max()
    pad = args.boundary_pad_m
    # Boundary corners in the SAME local-xy frame as the recon points, then
    # reprojected to lat/lon anchored at (x=0,y=0) -> (--anchor-lat/-lon).
    # This is what keeps centroid_xy and elevation_grid in one frame — see
    # the module docstring above for why that's not true in general.
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

    elevation_grid, origin_xy, smoothing_used = build_elevation_grid(
        xy, elevation, args.dem_resolution_m)
    print(f'[test] elevation grid: {elevation_grid.shape}, '
          f'origin_xy={origin_xy}, smoothing={smoothing_used}')

    centroid_xy = field_centroid_xy(corners_ll)
    lat0, lon0 = corners_ll[0]
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
