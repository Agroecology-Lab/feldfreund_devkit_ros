#!/usr/bin/env python3
"""agri_field_to_terrain.py — whole-field terrain for the Sowbot Gazebo sim.

Turns one field's `elevation_grid/<field>.ply` into a terrain mesh that drops
into the ground model (`models/ground/mesh/terrain.{obj,stl}`). The mesh covers
the entire field, follows the real field boundary (polygons_2D/<field>.xml),
and is scaled uniformly to convert km-scale fields to sim metres.

Usage:
    python3 agri_field_to_terrain.py --dataset Agri-Field-Dataset \
        --field 27 --out models/ground
"""

import argparse
import re
from pathlib import Path

import numpy as np
from scipy.interpolate import RegularGridInterpolator
from shapely.geometry import Point, Polygon
from stl import mesh as stl_mesh

SCALE = 0.01  # 1:100 — km-scale field down to sim metres (all axes)


def read_elevation_grid(ply_path):
    """Read a binary elevation_grid PLY into (xs, ys, Z) grid arrays."""
    with open(ply_path, 'rb') as f:
        header = []
        while True:
            line = f.readline()
            if not line:
                break
            header.append(line.decode().strip())
            if line.startswith(b'end_header'):
                break
        n_verts = next(int(h.split()[-1]) for h in header
                       if h.startswith('element vertex'))
        rest = f.read()
    verts = np.frombuffer(rest, dtype='<f4', count=n_verts * 3).reshape(n_verts, 3)
    xs = np.unique(verts[:, 0])
    ys = np.unique(verts[:, 1])
    Z = np.full((len(ys), len(xs)), np.nan, dtype=np.float64)
    Z[np.searchsorted(ys, verts[:, 1]), np.searchsorted(xs, verts[:, 0])] = verts[:, 2]
    return xs, ys, Z


def read_field_polygon(dataset, field):
    """Load the field's boundary polygon from polygons_2D/<field>.xml."""
    xml = Path(dataset) / 'polygons_2D' / f'field{field}.xml'
    if not xml.exists():
        raise SystemExit(f"ERROR: {xml} not found")
    verts = [(float(x), float(y)) for x, y in re.findall(
        r'<x>([-\d.]+)</x>\s*<y>([-\d.]+)</y>', xml.read_text())]
    if len(verts) < 3:
        raise SystemExit(f"ERROR: no polygon vertices parsed from {xml}")
    return Polygon(verts)


def build_whole_field_mesh(interp, xs, ys, polygon):
    """Build a mesh covering the entire field polygon, scaled uniformly.

    Uses the native grid points inside the polygon plus densified boundary
    points, Delaunay-triangulated and filtered to the polygon so the terrain
    follows the real field shape.
    """
    from scipy.spatial import Delaunay
    xx, yy = np.meshgrid(xs, ys)
    interior = [(x, y) for x, y in zip(xx.ravel(), yy.ravel())
                if polygon.contains(Point(x, y))]
    spacing = np.median(np.diff(xs))
    boundary = set()
    coords = list(polygon.exterior.coords)[:-1]
    for a, b in zip(coords, coords[1:] + coords[:1]):
        dist = np.hypot(b[0] - a[0], b[1] - a[1])
        n = max(2, int(dist / spacing) + 1)
        for k in range(n):
            boundary.add((a[0] + (b[0] - a[0]) * k / n,
                          a[1] + (b[1] - a[1]) * k / n))
    pts = np.array(interior + list(boundary))
    zz = interp((pts[:, 1], pts[:, 0]))
    tri = Delaunay(pts)
    centroids = pts[tri.simplices].mean(axis=1)
    keep = np.array([polygon.contains(Point(c)) for c in centroids])
    cells = tri.simplices[keep]
    vertices = np.column_stack([pts * SCALE, zz * SCALE])
    # UVs: map the field XY onto the texture across the field bounds.
    lo = pts.min(axis=0)
    rng = pts.max(axis=0) - lo
    uvs = (pts - lo) / rng
    return vertices, uvs, cells


def center_and_shift(vertices, z_ref):
    """Centre XY on origin and shift Z so the reference height is at 0."""
    vertices[:, :2] -= np.mean(vertices[:, :2], axis=0)
    vertices[:, 2] -= z_ref


def calculate_normals(vertices, faces):
    normals = np.zeros_like(vertices)
    for face in faces:
        v0, v1, v2 = vertices[face]
        fn = np.cross(v1 - v0, v2 - v0)
        length = np.linalg.norm(fn)
        if length > 0:
            normals[face] += fn / length
    lengths = np.linalg.norm(normals, axis=1, keepdims=True)
    lengths[lengths == 0] = 1
    return normals / lengths


def write_obj(path, vertices, uvs, normals, faces):
    with open(path, 'w') as f:
        f.write("# Terrain mesh - Agri-Field-Dataset\n")
        for v in vertices:
            f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
        for uv in uvs:
            f.write(f"vt {uv[0]:.6f} {uv[1]:.6f}\n")
        for n in normals:
            f.write(f"vn {n[0]:.6f} {n[1]:.6f} {n[2]:.6f}\n")
        for face in faces:
            f.write(f"f {face[0]+1}/{face[0]+1}/{face[0]+1}"
                    f" {face[1]+1}/{face[1]+1}/{face[1]+1}"
                    f" {face[2]+1}/{face[2]+1}/{face[2]+1}\n")


def write_stl(path, vertices, faces):
    terrain = stl_mesh.Mesh(np.zeros(len(faces), dtype=stl_mesh.Mesh.dtype))
    for i, f in enumerate(faces):
        for j in range(3):
            terrain.vectors[i][j] = vertices[f[j]]
    terrain.save(str(path))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--dataset', required=True,
                    help='Agri-Field-Dataset root (dir containing elevation_grid/)')
    ap.add_argument('--field', type=int, required=True,
                    help='Field number (e.g. 27 -> elevation_grid/field27.ply)')
    ap.add_argument('--out', required=True,
                    help='Ground model dir to write mesh/terrain.obj + .stl')
    args = ap.parse_args()

    ply = Path(args.dataset) / 'elevation_grid' / f'field{args.field}.ply'
    if not ply.exists():
        raise SystemExit(f"ERROR: {ply} not found")

    polygon = read_field_polygon(args.dataset, args.field)
    xs, ys, Z = read_elevation_grid(ply)
    print(f"Field {args.field}: grid {len(xs)}x{len(ys)} "
          f"spacing ~{np.median(np.diff(xs)):.0f} m, "
          f"elevation {np.nanmin(Z):.0f}..{np.nanmax(Z):.0f} m")

    interp = RegularGridInterpolator(
        (ys, xs), Z, method='linear', bounds_error=False, fill_value=None)
    vertices, uvs, faces = build_whole_field_mesh(interp, xs, ys, polygon)
    print(f"Whole field: {len(vertices)} verts, {len(faces)} faces "
          f"(scale x{SCALE})")

    # Put the field centroid (robot/crop spawn area) at height 0 so the
    # Forest3D plants (spawned at z~0) sit on the surface.
    z_ref = float(interp((polygon.centroid.y, polygon.centroid.x))) * SCALE
    center_and_shift(vertices, z_ref)
    normals = calculate_normals(vertices, faces)

    mesh_dir = Path(args.out) / 'mesh'
    mesh_dir.mkdir(parents=True, exist_ok=True)
    obj_path = mesh_dir / 'terrain.obj'
    stl_path = mesh_dir / 'terrain.stl'
    write_obj(obj_path, vertices, uvs, normals, faces)
    write_stl(stl_path, vertices, faces)

    print(f"Wrote {obj_path}")
    print(f"Wrote {stl_path}")
    print(f"Terrain: X={np.ptp(vertices[:,0]):.2f} "
          f"Y={np.ptp(vertices[:,1]):.2f} "
          f"Z={np.ptp(vertices[:,2]):.2f} m, "
          f"{len(vertices)} verts")


if __name__ == '__main__':
    main()
