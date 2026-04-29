"""
Place / Deform Children of a Flat UV Mesh onto a Curved 3D Source
==================================================================

Third script in the UV pipeline suite (paired with flatten_uv_to_geo.py
and unflatten_uv_to_geo.py).

Given:
  • A FLAT mesh (typically the "*_UV_FLAT" output from flatten_uv_to_geo.py)
    that has decoration objects parented under it as children — cylinders,
    spheres, custom hardware geometry, hole cutters, anything.
  • A SOURCE curved mesh (the original chair / object before flattening).

Produces a new null group beside the source containing copies of every
child, each one moved & oriented from its flat-space position to the
corresponding spot on the curved surface.

Four operating modes:

  MODE = "instance"   (default — recommended for many cutters / decorations)
    Each child becomes a live c4d.Oinstance pointing back to the original.
    Render Instance mode is enabled so hundreds of copies share one
    geometry cache (very memory-efficient). This means the placements
    stay LIVE: edit the original child (slide an axis node, change the
    cylinder height, swap geometry, modify a Cloner inside a Connect,
    etc.) and ALL placed copies update automatically. Same orientation
    behavior as "place".

  MODE = "place"
    Each child is cloned (baked, independent copy) and the clone is
    placed at the curved position oriented to surface normal. Child
    geometry is NOT distorted — a cylinder stays a cylinder. Use this
    when you want each placement editable independently of the master.

  MODE = "deform"
    Each child must be an editable polygon mesh. Every vertex of every
    child has its world-space position projected via UV → 3D, so the
    mesh wraps the surface curvature. Best for surface graphics,
    embossing, anything that should follow the curve geometrically.

  MODE = "all"
    Produces all three output groups simultaneously.

------------------------------------------------------------------------------
USAGE
------------------------------------------------------------------------------
  1. With your *_UV_FLAT mesh in the scene, build/parent your decoration
     objects as children of it (so their world-space positions land where
     you want them in UV space).
  2. Select the FLAT mesh AND the original curved SOURCE mesh.
     (Or select just the FLAT — the script will look for the source by
     stripping "_UV_FLAT" from the name.)
  3. Edit MODE below if you want deform / both instead of place.
  4. Extensions → Script Manager → Load Script → run.

------------------------------------------------------------------------------
HOW THE TRANSFORM WORKS (place mode)
------------------------------------------------------------------------------
For each child:
  • Read child world position; UV = (pos.x / SCALE, pos.z / SCALE).
  • Find source triangle whose UVs contain that UV point.
  • Compute curved 3D position via barycentric interpolation of the
    triangle's 3D vertex positions.
  • Surface frame at that point:
      N (normal) = face normal of the source triangle
      T_u       = direction in 3D along which U increases
      T_v       = N × T_u (orthogonalized)
  • Build new transform matrix combining the surface frame with the
    child's existing local rotation (so children rotated in flat space
    keep their relative orientation when placed).

------------------------------------------------------------------------------
LICENSE
------------------------------------------------------------------------------
MIT — free to use, modify, redistribute.

Author: Spenser Dickerson / SD Imaging  (sdimaging.art)
"""

import c4d
from c4d import documents
import time


# === Configuration ===========================================================

SCALE = 1000.0          # Must match the SCALE used in flatten_uv_to_geo.py
GRID_RES = 64           # Spatial grid resolution for UV → tri lookup

# "instance" : LIVE — output c4d.Oinstance objects pointing back to children.
#              Edits to the original propagate to all placements. Best for
#              many cutters / decorations where you want a master + variants.
# "place"    : BAKED — each child gets a rigid independent clone at curved pos.
# "deform"   : MESH — vertex-level surface deform (children must be poly meshes).
# "all"      : produce all three output groups simultaneously.
MODE = "instance"

# =============================================================================


def build_uv_tri_cache(src):
    """Same as unflatten_uv_to_geo.py: builds tri cache + UV grid for fast
    UV → 3D lookup. Each tri additionally stores its face normal and its
    UV-to-3D tangent vector along U."""
    uv_tag = src.GetTag(c4d.Tuvw)
    if not uv_tag:
        raise RuntimeError(f"'{src.GetName()}' has no UVW tag.")

    src_points = src.GetAllPoints()
    n_polys = src.GetPolygonCount()
    tris = []

    for i in range(n_polys):
        poly = src.GetPolygon(i)
        uv = uv_tag.GetSlow(i)
        ua, ub, uc, ud = uv["a"], uv["b"], uv["c"], uv["d"]
        pa, pb, pc, pd = src_points[poly.a], src_points[poly.b], src_points[poly.c], src_points[poly.d]
        is_tri = (poly.c == poly.d)

        for (uv0, uv1, uv2, p0, p1, p2) in (
            ((ua, ub, uc, pa, pb, pc),) if is_tri else
            ((ua, ub, uc, pa, pb, pc), (ua, uc, ud, pa, pc, pd))
        ) if False else (   # awkward inline trick replaced below
            [(ua, ub, uc, pa, pb, pc)]
            if is_tri else
            [(ua, ub, uc, pa, pb, pc), (ua, uc, ud, pa, pc, pd)]
        ):
            # Face normal of the 3D triangle
            edge1 = p1 - p0
            edge2 = p2 - p0
            n = edge1.Cross(edge2)
            ln = n.GetLength()
            if ln < 1e-12:
                continue   # degenerate
            n = n / ln

            # 3D tangent along U direction at this triangle:
            # solve for vector T_u such that going +1 in U gives T_u in 3D
            # (T_u, T_v) satisfy:  edge1 = (u1-u0) T_u + (v1-v0) T_v
            #                      edge2 = (u2-u0) T_u + (v2-v0) T_v
            du1, dv1 = uv1.x - uv0.x, uv1.y - uv0.y
            du2, dv2 = uv2.x - uv0.x, uv2.y - uv0.y
            det = du1 * dv2 - du2 * dv1
            if abs(det) < 1e-12:
                # Degenerate UVs — fall back to edge1 direction
                T_u = edge1.GetNormalized() if edge1.GetLength() > 1e-12 else c4d.Vector(1, 0, 0)
            else:
                T_u = (edge1 * dv2 - edge2 * dv1) * (1.0 / det)
                if T_u.GetLength() < 1e-12:
                    T_u = c4d.Vector(1, 0, 0)
                else:
                    T_u = T_u.GetNormalized()

            # Orthogonalize: T_v = N × T_u
            T_v = n.Cross(T_u).GetNormalized()
            # Re-orthogonalize T_u to ensure perpendicular to N
            T_u = T_v.Cross(n).GetNormalized()

            tris.append({
                "uv0": uv0, "uv1": uv1, "uv2": uv2,
                "p0": p0, "p1": p1, "p2": p2,
                "u_min": min(uv0.x, uv1.x, uv2.x),
                "u_max": max(uv0.x, uv1.x, uv2.x),
                "v_min": min(uv0.y, uv1.y, uv2.y),
                "v_max": max(uv0.y, uv1.y, uv2.y),
                "N": n, "Tu": T_u, "Tv": T_v,
            })

    grid = [[[] for _ in range(GRID_RES)] for _ in range(GRID_RES)]
    for ti, t in enumerate(tris):
        u_lo = max(0, int(t["u_min"] * GRID_RES))
        u_hi = min(GRID_RES - 1, int(t["u_max"] * GRID_RES))
        v_lo = max(0, int(t["v_min"] * GRID_RES))
        v_hi = min(GRID_RES - 1, int(t["v_max"] * GRID_RES))
        for vy in range(v_lo, v_hi + 1):
            for ux in range(u_lo, u_hi + 1):
                grid[vy][ux].append(ti)
    return tris, grid


def barycentric_2d(u, v, uv0, uv1, uv2):
    den = ((uv1.y - uv2.y) * (uv0.x - uv2.x) +
           (uv2.x - uv1.x) * (uv0.y - uv2.y))
    if abs(den) < 1e-12:
        return None
    w0 = ((uv1.y - uv2.y) * (u - uv2.x) +
          (uv2.x - uv1.x) * (v - uv2.y)) / den
    w1 = ((uv2.y - uv0.y) * (u - uv2.x) +
          (uv0.x - uv2.x) * (v - uv2.y)) / den
    return (w0, w1, 1.0 - w0 - w1)


def find_tri_at_uv(u, v, tris, grid):
    """Return (tri, bary) or (None, None)."""
    if not (0.0 <= u <= 1.0 and 0.0 <= v <= 1.0):
        return None, None
    cu = max(0, min(GRID_RES - 1, int(u * GRID_RES)))
    cv = max(0, min(GRID_RES - 1, int(v * GRID_RES)))
    for radius in (0, 1, 2):
        for dv in range(-radius, radius + 1):
            for du in range(-radius, radius + 1):
                if radius > 0 and abs(du) != radius and abs(dv) != radius:
                    continue
                ccu, ccv = cu + du, cv + dv
                if not (0 <= ccu < GRID_RES and 0 <= ccv < GRID_RES):
                    continue
                for ti in grid[ccv][ccu]:
                    t = tris[ti]
                    if u < t["u_min"] or u > t["u_max"] or v < t["v_min"] or v > t["v_max"]:
                        continue
                    bary = barycentric_2d(u, v, t["uv0"], t["uv1"], t["uv2"])
                    if bary is None:
                        continue
                    w0, w1, w2 = bary
                    eps = 1e-5
                    if w0 >= -eps and w1 >= -eps and w2 >= -eps:
                        return t, bary
    return None, None


def collect_children(parent):
    """Return all placement targets under `parent`.

    Recursion rule: Null objects are treated as TRANSPARENT organizing
    wrappers — the script descends through them and picks up their
    children as placement candidates. Any non-Null object (Cube, Cylinder,
    Sphere, Instance, Polygon mesh, primitive, etc.) is treated as a
    placement candidate and is NOT descended into. This means:

      Flat
        └── Group_Null         ← descended into (transparent wrapper)
              ├── Inst_001     ← placement candidate
              ├── Inst_002     ← placement candidate
              └── Sub_Null     ← also descended into
                    └── Inst_003   ← placement candidate

    If your "master" is a compound assembly (e.g. a Connect with a Cloner
    inside), wrap it in a non-Null parent (or bake it to a polygon mesh
    via 'Current State to Object') so the script treats the whole assembly
    as one placement unit. Or simpler: keep the compound master OUTSIDE
    the flat hierarchy entirely, and use Instance objects as flat
    children that link to the master.

    For Cloners with templates inside: bake the Cloner first
    ('Current State to Object'), so the cloner outputs become real
    objects under the flat mesh."""
    result = []

    def recurse(obj):
        if obj.GetType() == c4d.Onull:
            # Transparent organizing wrapper — descend
            ch = obj.GetDown()
            while ch:
                recurse(ch)
                ch = ch.GetNext()
        else:
            # Real placement target
            result.append(obj)

    ch = parent.GetDown()
    while ch:
        recurse(ch)
        ch = ch.GetNext()
    return result


def get_world_placement_pos(child):
    """Returns the object's effective world-space position for UV lookup.

    Uses bbox center (Mg * Mp) — this is robust to:
      • Primitives (Mp ≈ 0, so result = Mg.off, the pivot)
      • Centered poly meshes (Mp ≈ 0, same as above)
      • Frozen-coord polys (Mp != 0, world position encoded in local verts;
        bbox center recovers the actual world position)
    """
    return child.GetMg() * child.GetMp()


def compute_curved_transform(child, tris, grid):
    """Shared logic: returns (matrix, src_world_pos, error) where matrix
    places `child` on the curved surface oriented to the surface normal,
    src_world_pos is the source's world-space placement origin (used for
    re-centering polygon meshes), and error is None on success."""
    child_mg = child.GetMg()
    src_world_pos = get_world_placement_pos(child)
    u = src_world_pos.x / SCALE
    v = src_world_pos.z / SCALE

    t, bary = find_tri_at_uv(u, v, tris, grid)
    if t is None:
        return None, src_world_pos, "outside any UV island"

    w0, w1, w2 = bary
    curved_pos = t["p0"] * w0 + t["p1"] * w1 + t["p2"] * w2

    R_surf = c4d.Matrix(off=c4d.Vector(0, 0, 0),
                        v1=t["Tu"], v2=t["N"], v3=t["Tv"])
    R_child = c4d.Matrix(off=c4d.Vector(0, 0, 0),
                         v1=child_mg.v1, v2=child_mg.v2, v3=child_mg.v3)
    R_combined = R_surf * R_child

    new_mg = c4d.Matrix(off=curved_pos,
                        v1=R_combined.v1, v2=R_combined.v2, v3=R_combined.v3)
    return new_mg, src_world_pos, None


def clone_polygon_recentered(child, new_mg, src_world_pos):
    """Clone a polygon mesh with vertices RE-CENTERED around the source's
    world-space bbox center. Each new local vertex = (source world vertex
    position - source world centroid). Robust to frozen-coord polys whose
    geometry encodes world position in local verts."""
    src_pts = child.GetAllPoints()
    src_mg = child.GetMg()
    n_polys = child.GetPolygonCount()
    n_pts = len(src_pts)

    # Re-center: world vertex - world centroid → local offset around new origin
    new_pts = [src_mg * src_pts[i] - src_world_pos for i in range(n_pts)]

    out = c4d.PolygonObject(n_pts, n_polys)
    out.SetAllPoints(new_pts)
    for i in range(n_polys):
        out.SetPolygon(i, child.GetPolygon(i))
    out.Message(c4d.MSG_UPDATE)
    out.MakeTag(c4d.Tphong)
    out.SetMg(new_mg)
    out.SetName(child.GetName() + "_curved")
    return out


def is_frozen_coord_poly(child, threshold=1e-3):
    """Returns True if child is a polygon mesh whose geometry is encoded in
    local vertex coords with the bbox center far from the object's origin
    (typical of post-Split/Disconnect pieces). Threshold is in scene units."""
    if child.GetType() != c4d.Opolygon:
        return False
    return child.GetMp().GetLength() > threshold


def place_child_rigid(child, tris, grid):
    """Mode A: clone the child rigidly with new position + surface-aligned matrix.

    For polygon meshes: re-centers vertices around the source bbox center,
    so post-Split pieces with frozen world-space coords work correctly.
    For primitives: simple GetClone + SetMg (their geometry is already
    centered on Mg.off)."""
    new_mg, src_world_pos, err = compute_curved_transform(child, tris, grid)
    if new_mg is None:
        return None, err

    if child.GetType() == c4d.Opolygon:
        # Always re-center for poly meshes — handles both centered and frozen coord cases
        return clone_polygon_recentered(child, new_mg, src_world_pos), None
    else:
        clone = child.GetClone(c4d.COPYFLAGS_NONE)
        clone.SetMg(new_mg)
        clone.SetName(child.GetName() + "_curved")
        return clone, None


def place_child_as_instance(child, tris, grid):
    """Mode "instance": create a LIVE c4d.Oinstance linked to the child,
    placed at curved position with surface-aligned orientation. Render
    Instance mode is enabled for memory-efficient many-copies workflows.

    Caveat: instances of frozen-coord polygon meshes (objects whose vertex
    data is in world space, not local) render incorrectly because the
    instance applies its new transform to those world coords. For such
    sources we fall back to the rigid-clone-with-recentering path so the
    geometry actually lands at the target. A console warning is printed."""
    new_mg, src_world_pos, err = compute_curved_transform(child, tris, grid)
    if new_mg is None:
        return None, err

    if is_frozen_coord_poly(child):
        # Can't instance frozen-coord polys cleanly — fall back to clone
        return clone_polygon_recentered(child, new_mg, src_world_pos), \
               "frozen-coord poly: cloned with re-centering instead of instancing"

    inst = c4d.BaseObject(c4d.Oinstance)
    inst[c4d.INSTANCEOBJECT_LINK] = child
    try:
        inst[c4d.INSTANCEOBJECT_RENDERINSTANCE] = True
    except Exception:
        pass
    inst.SetMg(new_mg)
    inst.SetName(child.GetName() + "_inst")
    return inst, None


def deform_child_mesh(child, tris, grid):
    """Mode B: clone the child as a polygon mesh, project every vertex via UV → 3D."""
    if child.GetType() != c4d.Opolygon:
        return None, f"not a polygon mesh ({child.GetTypeName()}); use 'Current State to Object' first"

    child_mg = child.GetMg()
    src_pts = child.GetAllPoints()
    n_pts = len(src_pts)

    new_pts = [None] * n_pts
    n_outside = 0
    for i, p_local in enumerate(src_pts):
        p_world = child_mg * p_local
        u = p_world.x / SCALE
        v = p_world.z / SCALE
        t, bary = find_tri_at_uv(u, v, tris, grid)
        if t is None:
            new_pts[i] = p_world
            n_outside += 1
        else:
            w0, w1, w2 = bary
            new_pts[i] = t["p0"] * w0 + t["p1"] * w1 + t["p2"] * w2

    n_polys = child.GetPolygonCount()
    out = c4d.PolygonObject(n_pts, n_polys)
    out.SetAllPoints(new_pts)
    for i in range(n_polys):
        out.SetPolygon(i, child.GetPolygon(i))
    out.Message(c4d.MSG_UPDATE)
    out.MakeTag(c4d.Tphong)
    out.SetName(child.GetName() + "_deformed")
    # Output is in world space (no transform)
    return out, (f"{n_outside} verts outside any UV island" if n_outside else None)


def main():
    if MODE not in ("instance", "place", "deform", "all"):
        c4d.gui.MessageDialog(
            f"Invalid MODE='{MODE}'. Use 'instance', 'place', 'deform', or 'all'."
        )
        return

    doc = documents.GetActiveDocument()
    selected = doc.GetActiveObjects(c4d.GETACTIVEOBJECTFLAGS_NONE) or []

    flat = src = None
    if len(selected) >= 2 and all(s.GetType() == c4d.Opolygon for s in selected[:2]):
        flat, src = selected[0], selected[1]
    elif len(selected) == 1 and selected[0].GetType() == c4d.Opolygon:
        flat = selected[0]
        if flat.GetName().endswith("_UV_FLAT"):
            target = flat.GetName()[:-len("_UV_FLAT")]
            o = doc.GetFirstObject()
            while o:
                if o.GetName() == target and o.GetType() == c4d.Opolygon:
                    src = o; break
                o = o.GetNext()

    if not flat or not src:
        c4d.gui.MessageDialog(
            "Select TWO polygon objects in this order:\n"
            "  1. The FLAT mesh (with decorations parented under it)\n"
            "  2. The original curved SOURCE mesh\n\n"
            "Or select just the FLAT and the script will look for the source\n"
            "by stripping '_UV_FLAT' from the name."
        )
        return

    children = collect_children(flat)
    if not children:
        c4d.gui.MessageDialog(f"'{flat.GetName()}' has no children to place.")
        return

    print("=" * 64)
    print(f"Place / Deform Children — Mode: {MODE.upper()}")
    print("=" * 64)
    print(f"Flat:     {flat.GetName()}  ({len(children)} children)")
    print(f"Source:   {src.GetName()}")
    print(f"SCALE:    {SCALE}")

    t0 = time.time()
    tris, grid = build_uv_tri_cache(src)
    t1 = time.time()
    print(f"  Built UV tri cache: {len(tris)} tris in {t1 - t0:.2f}s")

    out_groups = []

    if MODE in ("instance", "all"):
        group = c4d.BaseObject(c4d.Onull)
        group.SetName(f"{flat.GetName()}_INSTANCED_ON_CURVED")
        doc.InsertObject(group)
        n_ok = 0
        n_failed = 0
        for ch in children:
            inst, err = place_child_as_instance(ch, tris, grid)
            if inst is None:
                n_failed += 1
                continue
            inst.InsertUnder(group)
            n_ok += 1
        out_groups.append((group, n_ok, n_failed, "live instance(s)"))

    if MODE in ("place", "all"):
        group = c4d.BaseObject(c4d.Onull)
        group.SetName(f"{flat.GetName()}_PLACED_ON_CURVED")
        doc.InsertObject(group)
        n_placed = 0
        n_failed = 0
        for ch in children:
            clone, err = place_child_rigid(ch, tris, grid)
            if clone is None:
                n_failed += 1
                continue
            clone.InsertUnder(group)
            n_placed += 1
        out_groups.append((group, n_placed, n_failed, "placed (rigid clone)"))

    if MODE in ("deform", "all"):
        group = c4d.BaseObject(c4d.Onull)
        group.SetName(f"{flat.GetName()}_DEFORMED_ON_CURVED")
        doc.InsertObject(group)
        n_deformed = 0
        n_failed = 0
        warnings = 0
        for ch in children:
            mesh, err = deform_child_mesh(ch, tris, grid)
            if mesh is None:
                n_failed += 1
                continue
            mesh.InsertUnder(group)
            n_deformed += 1
            if err:
                warnings += 1
        out_groups.append((group, n_deformed, n_failed,
                           f"deformed (mesh){' — ' + str(warnings) + ' had outside-UV verts' if warnings else ''}"))

    c4d.EventAdd()
    t2 = time.time()

    print()
    for group, n_ok, n_fail, label in out_groups:
        print(f"  → {group.GetName()}: {n_ok} {label}, {n_fail} skipped")
    print(f"  TOTAL: {t2 - t0:.2f}s")
    print("=" * 64)


if __name__ == "__main__":
    main()
