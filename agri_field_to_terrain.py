#!/usr/bin/env python3
"""agri_field_to_terrain.py — whole-field terrain for the Sowbot Gazebo sim.

Turns one field's `surfaces_3D/<field>.ply` into a terrain mesh that drops
into the ground model (`models/ground/mesh/terrain.{obj,stl}`). The mesh covers
the entire field, follows the real field boundary (polygons_2D/<field>.xml),
and is scaled uniformly to convert km-scale fields to sim metres.

`surfaces_3D` is the dense ~25 m point cloud; it is resampled onto a regular
grid (the decimation) before Delaunay triangulation so the exported mesh stays
light enough for DART collision.

Usage:
    python3 agri_field_to_terrain.py --dataset Agri-Field-Dataset \
        --field 27 --out models/ground
"""

import argparse
import re
from pathlib import Path

import numpy as np
from scipy.interpolate import LinearNDInterpolator
from shapely.geometry import Point, Polygon
from stl import mesh as stl_mesh

SCALE = 0.01  # 1:100 — km-scale field down to sim metres (all axes)
GRID = 200.0  # resample spacing in real metres (decimation of surfaces_3D)


def read_surface(ply_path):
    """Read the vertex section of a surfaces_3D PLY into an (N, 3) array."""
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
    return np.frombuffer(rest, dtype='<f4', count=n_verts * 3).reshape(n_verts, 3)


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


def sample_field(surface, polygon, grid):
    """Sample the surface point cloud on a regular grid inside the polygon.

    Returns (points, z) where points are the (N, 2) XY positions. Densified
    boundary points are included so the mesh edge follows the field shape.
    """
    xmin, ymin, xmax, ymax = polygon.bounds
    gx = np.arange(xmin, xmax, grid)
    gy = np.arange(ymin, ymax, grid)
    xx, yy = np.meshgrid(gx, gy)
    pts = np.column_stack([xx.ravel(), yy.ravel()])
    inside = np.array([polygon.contains(Point(p)) for p in pts])
    pts = pts[inside]

    spacing = np.median(np.diff(gx))
    boundary = set()
    coords = list(polygon.exterior.coords)[:-1]
    for a, b in zip(coords, coords[1:] + coords[:1]):
        dist = np.hypot(b[0] - a[0], b[1] - a[1])
        n = max(2, int(dist / spacing) + 1)
        for k in range(n):
            boundary.add((a[0] + (b[0] - a[0]) * k / n,
                          a[1] + (b[1] - a[1]) * k / n))
    boundary = np.array(list(boundary))
    points = np.vstack([pts, boundary])

    interp = LinearNDInterpolator(surface[:, :2], surface[:, 2])
    z = interp(points)
    # LinearNDInterpolator returns NaN outside the point cloud; fill with
    # nearest-neighbour for the few boundary points that may fall in a gap.
    if np.isnan(z).any():
        good = ~np.isnan(z)
        from scipy.interpolate import NearestNDInterpolator
        nn = NearestNDInterpolator(points[good], z[good])
        z[~good] = nn(points[~good])
    return points, z


def triangulate(points, z, polygon):
    """Delaunay-triangulate the sampled points, keeping triangles inside the
    field polygon. Returns (vertices, uvs, faces) in real metres."""
    from scipy.spatial import Delaunay
    tri = Delaunay(points)
    centroids = points[tri.simplices].mean(axis=1)
    keep = np.array([polygon.contains(Point(c)) for c in centroids])
    faces = tri.simplices[keep]
    vertices = np.column_stack([points * SCALE, z * SCALE])
    lo = points.min(axis=0)
    rng = points.max(axis=0) - lo
    uvs = (points - lo) / rng
    return vertices, uvs, faces


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
                    help='Agri-Field-Dataset root (dir containing surfaces_3D/)')
    ap.add_argument('--field', type=int, required=True,
                    help='Field number (e.g. 27 -> surfaces_3D/field27.ply)')
    ap.add_argument('--out', required=True,
                    help='Ground model dir to write mesh/terrain.obj + .stl')
    ap.add_argument('--grid', type=float, default=GRID,
                    help='Resample spacing in real metres (decimation; '
                         f'default {GRID:.0f} m)')
    args = ap.parse_args()

    ply = Path(args.dataset) / 'surfaces_3D' / f'field{args.field}.ply'
    if not ply.exists():
        raise SystemExit(f"ERROR: {ply} not found")

    polygon = read_field_polygon(args.dataset, args.field)
    surface = read_surface(ply)
    print(f"Field {args.field}: {len(surface):,} surface points, "
          f"elevation {surface[:, 2].min():.0f}..{surface[:, 2].max():.0f} m")

    points, z = sample_field(surface, polygon, args.grid)
    vertices, uvs, faces = triangulate(points, z, polygon)
    print(f"Whole field: {len(vertices):,} verts, {len(faces):,} faces "
          f"(resampled at {args.grid:.0f} m, scale x{SCALE})")

    # Put the field centroid (robot/crop spawn area) at height 0 so the
    # Forest3D plants (spawned at z~0) sit on the surface.
    from scipy.interpolate import NearestNDInterpolator
    nn = NearestNDInterpolator(surface[:, :2], surface[:, 2])
    z_ref = float(nn((polygon.centroid.x, polygon.centroid.y))) * SCALE
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
          f"{len(vertices):,} verts")


if __name__ == '__main__':
    main()
