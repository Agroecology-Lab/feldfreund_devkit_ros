#!/usr/bin/env python3
"""check_model.py — static mechanical sanity checks for a robot URDF/xacro.

Usage:
    python3 check_model.py [path/to/robot.xacro | path/to/robot.urdf]

Runs, in order:
    1. Parse + kinematic-tree validation (single root, joint axes).
    2. Collision-AABB overlap detection between links, classified by the
       joint path between them (rigid assembly vs. jointed vs. adjacent).
    3. Mass / centre-of-mass / static stability (support polygon, tip-over
       angles).
    4. Inertia plausibility (vs. m/12(a^2+b^2) box ballpark, positive
       definiteness).

Exit code: 0 = no errors, 1 = errors found, 2 = usage error.

Collision overlaps are only "hard" errors for bodies linked purely by fixed
joints (they occupy the same rigid body and would fight if self_collide were
enabled); overlaps across moving joints are reported for information.
"""

import math
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

WARN_ONLY = "--warn-only" in sys.argv
if WARN_ONLY:
    sys.argv.remove(WARN_ONLY)


def rot(rpy):
    r, p, y = (float(v) for v in rpy)
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def hmat(xyz, rpy):
    t = np.eye(4)
    t[:3, :3] = rot(rpy)
    t[:3, 3] = [float(v) for v in xyz]
    return t


def corner_aabb(corners):
    return np.min(corners, axis=0), np.max(corners, axis=0)


def aabb_overlap(a, b, tol=1e-4):
    (ax0, ay0, az0), (ax1, ay1, az1) = a
    (bx0, by0, bz0), (bx1, by1, bz1) = b
    return ((ax1 - tol > bx0 and bx1 - tol > ax0)
            and (ay1 - tol > by0 and by1 - tol > ay0)
            and (az1 - tol > bz0 and bz1 - tol > az0))


def overlap_depth(a, b):
    (ax0, ay0, az0), (ax1, ay1, az1) = a
    (bx0, by0, bz0), (bx1, by1, bz1) = b
    return (
        min(ax1, bx1) - max(ax0, bx0),
        min(ay1, by1) - max(ay0, by0),
        min(az1, bz1) - max(az0, bz0),
    )


def shape_corners(collision):
    geo = collision.find("geometry")
    box = geo.find("box")
    if box is not None:
        sx, sy, sz = (float(v) for v in box.get("size").split())
        half = np.array([sx, sy, sz]) / 2.0
        corners = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1)
                            for z in (-1, 1)]) * half
    else:
        cyl = geo.find("cylinder")
        if cyl is not None:
            r = float(cyl.get("radius"))
            h = float(cyl.get("length"))
            half = np.array([r, r, h / 2.0])
            corners = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1)
                                for z in (-1, 1)]) * half
        else:
            return None
    origin = collision.find("origin")
    xyz = origin.get("xyz", "0 0 0").split() if origin is not None else ["0", "0", "0"]
    rpy = origin.get("rpy", "0 0 0").split() if origin is not None else ["0", "0", "0"]
    m = hmat(xyz, rpy)
    return (m[:3, :3] @ corners.T).T + m[:3, 3]


def parse(urdf_path):
    import xml.etree.ElementTree as ET
    return ET.parse(urdf_path).getroot()


def load_urdf(src):
    src = Path(src)
    if src.suffix in (".xacro",):
        with tempfile.NamedTemporaryFile(suffix=".urdf") as tmp:
            subprocess.run(["xacro", str(src)], check=True, stdout=tmp,
                           stderr=subprocess.DEVNULL)
            tmp.flush()
            return parse(tmp.name)
    return parse(src)


def build_tree(root):
    links = {l.get("name"): l for l in root.findall("link")}
    joints = {}
    for j in root.findall("joint"):
        joints[j.find("child").get("link")] = (
            j.find("parent").get("link"), j.get("type"), j)
    return links, joints


def path_to_root(name, joints):
    path = []
    while name in joints:
        parent, jtype, j = joints[name]
        path.append((name, parent, jtype, j))
        name = parent
    return name, path


def world_of(name, joints, link, links):
    t = np.eye(4)
    cur = name
    while cur in joints:
        parent, _, j = joints[cur]
        origin = j.find("origin")
        xyz = origin.get("xyz", "0 0 0").split() if origin is not None else ["0", "0", "0"]
        rpy = origin.get("rpy", "0 0 0").split() if origin is not None else ["0", "0", "0"]
        t = hmat(xyz, rpy) @ t
        cur = parent
    return t


def inertia_ballpark(link, mass):
    coll = link.find("collision")
    if coll is None:
        return None
    geo = coll.find("geometry")
    box = geo.find("box")
    if box is not None:
        L, W, H = (float(v) for v in box.get("size").split())
        return mass / 12.0 * (W * W + H * H), \
            mass / 12.0 * (L * L + H * H), \
            mass / 12.0 * (L * L + W * W)
    cyl = geo.find("cylinder")
    if cyl is not None:
        r = float(cyl.get("radius"))
        h = float(cyl.get("length"))
        return mass * (3 * r * r + h * h) / 12.0, \
            mass * r * r / 2.0, \
            mass * (3 * r * r + h * h) / 12.0
    return None


def check_structure(root, links):
    joints = {}
    ok = True
    print(f"== STRUCTURE ==")
    for j in root.findall("joint"):
        child_el, parent_el = j.find("child"), j.find("parent")
        child = child_el.get("link") if child_el is not None else None
        parent = parent_el.get("link") if parent_el is not None else None
        if child is None or parent is None:
            print(f"  ERROR: joint {j.get('name')} missing parent or child link")
            ok = False
            continue
        if child in joints:
            print(f"  ERROR: link {child} has multiple parent joints "
                  f"({joints[child][0]} and {parent})")
            ok = False
            continue
        joints[child] = (parent, j.get("type"), j)

    roots = [n for n in links if n not in joints]
    print(f"  links: {len(links)}, joints: {len(joints)}")
    if len(roots) != 1:
        print(f"  ERROR: expected exactly one root link, found {roots}")
        return False
    root_link = roots[0]
    print(f"  root link: {root_link}")

    # Every link must be reachable from the single root by walking parents,
    # with no cycles. world_of()/path_to_root() walk `child -> parent` until
    # they fall off the joints dict, so an unreachable link or a cycle among
    # non-root links would otherwise loop forever or silently misplace parts.
    for name in links:
        seen = set()
        cur = name
        while cur in joints:
            if cur in seen:
                print(f"  ERROR: cycle detected in kinematic chain at {cur}")
                ok = False
                break
            seen.add(cur)
            cur = joints[cur][0]
        else:
            if cur != root_link:
                print(f"  ERROR: link {name} does not resolve to root "
                      f"{root_link} (found {cur} — is it a missing link?)")
                ok = False

    for child, (parent, jtype, j) in joints.items():
        if parent not in links:
            print(f"  ERROR: joint ->{child} references missing parent {parent}")
            ok = False
        if jtype == "fixed":
            continue
        axis = j.find("axis")
        if axis is None:
            print(f"  ERROR: joint {child} ({jtype}) has no <axis>")
            ok = False
        else:
            a = np.array([float(v) for v in axis.get("xyz").split()])
            if np.allclose(a, 0):
                print(f"  ERROR: joint {child} has zero-length axis")
                ok = False
    return ok


def check_overlaps(root, links, joints):
    print(f"\n== COLLISION AABB OVERLAPS ==")
    aabbs = {}
    poses = {}
    for name in links:
        t = world_of(name, joints, None, links)
        poses[name] = t
        for ci, coll in enumerate(links[name].findall("collision")):
            corners = shape_corners(coll)
            if corners is None:
                continue
            world_corners = (t[:3, :3] @ corners.T).T + t[:3, 3]
            aabbs[(name, ci)] = corner_aabb(world_corners)

    if not aabbs:
        print("  (no collision geometry)")
        return True

    names = sorted(aabbs)
    ok = True
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            (n1, c1), (n2, c2) = names[i], names[j]
            if not aabb_overlap(aabbs[names[i]], aabbs[names[j]]):
                continue
            dx, dy, dz = overlap_depth(aabbs[names[i]], aabbs[names[j]])
            root1, p1 = path_to_root(n1, joints)
            root2, p2 = path_to_root(n2, joints)
            a1 = [n1] + [parent for (_c, parent, _jt, _j) in p1]
            a2 = [n2] + [parent for (_c, parent, _jt, _j) in p2]
            a2_set = set(a2)
            idx1 = next(k for k, node in enumerate(a1) if node in a2_set)
            lca = a1[idx1]
            idx2 = a2.index(lca)
            # Only the joints strictly between n1 and n2 (via their lowest
            # common ancestor) determine whether these two links are part of
            # the same rigid body — shared ancestors above the LCA are
            # irrelevant to that question.
            path_jtypes = [jt for (_c, _p, jt, _j) in p1[:idx1]] + \
                          [jt for (_c, _p, jt, _j) in p2[:idx2]]
            all_fixed = bool(path_jtypes) and all(jt == "fixed" for jt in path_jtypes)
            non_fixed_count = sum(1 for jt in path_jtypes if jt != "fixed")
            if all_fixed:
                verdict = "RIGID OVERLAP (same body; masked while self_collide=off, breaks if enabled)"
                is_err = True
            elif non_fixed_count == 1:
                verdict = "adjacent moving joint (normally filtered)"
                is_err = False
            else:
                verdict = "moving-joint chain (filtered by self_collide=off)"
                is_err = False
            label = "ERROR" if is_err else "warn" if all_fixed else "note"
            print(f"  [{label}] {n1}(c{c1}) <-> {n2}(c{c2}) overlap "
                  f"({dx:.3f}, {dy:.3f}, {dz:.3f}) m — {verdict}")
            ok = ok and not is_err
    return ok


def convex_hull_2d(points):
    """Andrew's monotone chain. points: (N,2) array. Returns hull vertices, CCW."""
    pts = sorted(set(map(tuple, points)))
    if len(pts) <= 2:
        return np.array(pts)

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return np.array(lower[:-1] + upper[:-1])


def point_in_polygon(pt, poly):
    """Ray-casting point-in-polygon test. poly: (N,2) array, CCW or CW."""
    x, y = pt
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            x_int = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < x_int:
                inside = not inside
    return inside


def check_mass_cg(root, links, joints):
    print(f"\n== MASS / CG / STATIC STABILITY ==")
    total = 0.0
    cg = np.zeros(3)
    for name in links:
        iner = links[name].find("inertial")
        if iner is None:
            continue
        m = float(iner.find("mass").get("value"))
        t = world_of(name, joints, None, links)
        com = np.zeros(3)
        o = iner.find("origin")
        if o is not None and o.get("xyz"):
            com = np.array([float(v) for v in o.get("xyz").split()])
        total += m
        cg += m * (t[:3, :3] @ com + t[:3, 3])
    if total == 0:
        print("  ERROR: no link mass found")
        return False
    cg /= total
    print(f"  total mass   = {total:.1f} kg")
    print(f"  CG (root)    = ({cg[0]:+.4f}, {cg[1]:+.4f}, {cg[2]:+.4f}) m")

    ground_contacts = []
    for name in links:
        for ci, coll in enumerate(links[name].findall("collision")):
            corners = shape_corners(coll)
            if corners is None:
                continue
            t = world_of(name, joints, None, links)
            wc = (t[:3, :3] @ corners.T).T + t[:3, 3]
            if wc[:, 2].min() <= 1e-6:
                ground_contacts.append((name, wc[:, :2]))
    if ground_contacts:
        pts = np.vstack([p for _, p in ground_contacts])
        x0, y0 = pts[:, 0].min(), pts[:, 1].min()
        x1, y1 = pts[:, 0].max(), pts[:, 1].max()
        hl, hw = (x1 - x0) / 2, (y1 - y0) / 2
        cx, cy = (x1 + x0) / 2, (y1 + y0) / 2
        print(f"  support bbox = x +-{hl:.3f}, y +-{hw:.3f} (centre {cx:.3f}, {cy:.3f})")

        hull = convex_hull_2d(pts)
        # The bbox can pass (a rectangle) while the CG sits outside a
        # triangular/sparse contact polygon, e.g. 3-point contact — so the
        # real pass/fail is CG-in-hull, not CG-in-bbox.
        stable = len(hull) >= 3 and point_in_polygon((cg[0], cg[1]), hull)
        print(f"  support hull = {len(hull)} vertices")
        print(f"  CG inside support polygon: {'OK' if stable else 'FAIL'}")
        if stable and cg[2] > 1e-6:
            roll = math.degrees(math.atan(hw / cg[2]))
            pitch = math.degrees(math.atan(hl / cg[2]))
            print(f"  tip-over: roll {roll:.1f} deg, pitch {pitch:.1f} deg "
                  f"(>45 deg is comfortably stable, bbox-based estimate)")
        return stable
    print("  WARNING: no ground-level collision found")
    return False


def check_inertia(root, links):
    print(f"\n== INERTIA PLAUSIBILITY ==")
    ok = True
    for name in links:
        iner = links[name].find("inertial")
        if iner is None:
            continue
        mass = float(iner.find("mass").get("value"))
        ia = iner.find("inertia")
        I = {k: float(ia.get(k)) for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")}
        if min(I["ixx"], I["iyy"], I["izz"]) <= 0:
            print(f"  ERROR: {name}: non-positive diagonal inertia")
            ok = False
            continue
        diag = np.diag([I["ixx"], I["iyy"], I["izz"]])
        off = np.array([[0, I["ixy"], I["ixz"]],
                        [I["ixy"], 0, I["iyz"]],
                        [I["ixz"], I["iyz"], 0]])
        if not np.all(np.linalg.eigvals(diag + off) > 0):
            print(f"  ERROR: {name}: inertia matrix not positive-definite")
            ok = False
        ball = inertia_ballpark(links[name], mass)
        if ball is None:
            continue
        ratio = max(max(i / b, b / i) for i, b in zip((I["ixx"], I["iyy"], I["izz"]), ball) if b > 0)
        if ratio > 10:
            print(f"  note: {name}: inertia {ratio:.1f}x off from box ballpark "
                  f"({I['ixx']:.3f},{I['iyy']:.3f},{I['izz']:.3f} vs "
                  f"{ball[0]:.3f},{ball[1]:.3f},{ball[2]:.3f})")
    print("  all inertia matrices positive-definite" if ok else "  see errors above")
    return ok


def main():
    if len(sys.argv) != 2:
        print("usage: check_model.py <robot.urdf|robot.xacro> [--warn-only]",
              file=sys.stderr)
        return 2
    root = load_urdf(sys.argv[1])
    links, joints = build_tree(root)
    results = [
        check_structure(root, links),
        check_overlaps(root, links, joints),
        check_mass_cg(root, links, joints),
        check_inertia(root, links),
    ]
    n_fail = sum(1 for r in results if not r)
    print()
    if n_fail == 0:
        print("PASS — no mechanical errors detected.")
        return 0
    print(f"{n_fail} check(s) FAILED (details above).")
    if WARN_ONLY:
        return 0
    return 1


if __name__ == "__main__":
    sys.exit(main())
