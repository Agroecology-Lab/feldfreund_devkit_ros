"""Browser glue for contour rows: recon.csv -> reference contour -> contour swaths.

Mirrors ui_node._plan_contour_rows() without the ROS imports, so it runs in
Pyodide. Returns None when the field is too flat for a reference contour
(the caller falls back to straight rows).
"""
import os
import tempfile

import numpy as np
from devkit_f2c_planner.f2c_planner import _f2c_latlon_to_xy, _run_contour_f2c, field_centroid_xy
from devkit_ui.dem import build_elevation_grid, load_recon_points, select_reference_contour_latlon


def plan_contour(corners_ll, obstacle_rings, csv_text, tool_width, pad_m, headland_m,
                 snake, resolution_m=1.0):
    with tempfile.NamedTemporaryFile('w', suffix='.csv', delete=False, newline='') as f:
        f.write(csv_text)
        path = f.name
    try:
        _, elevation, latlon = load_recon_points(path)
    finally:
        os.unlink(path)
    lat0, lon0 = corners_ll[0]
    # Re-anchor on corners_ll[0] so the grid and the field centroid share one frame.
    xy = np.array([_f2c_latlon_to_xy(la, lo, lat0, lon0) for la, lo in latlon])
    grid, origin_xy, _ = build_elevation_grid(xy, elevation, resolution_m)
    ref = select_reference_contour_latlon(
        grid, resolution_m, origin_xy, field_centroid_xy(corners_ll), lat0, lon0)
    if ref is None:
        return None
    return _run_contour_f2c(corners_ll, obstacle_rings, ref, tool_width, pad_m, headland_m, snake)
