"""
Flat UV Geometry → Curved 3D Mesh  (reverse of flatten_uv_to_geo.py)
====================================================================

The companion script to flatten_uv_to_geo.py. Takes a modified flat mesh
laid out in UV space and projects every vertex back onto the original
curved 3D source mesh, using each flat vertex's position as its UV
coordinate (flat X = U×SCALE, flat Z = V×SCALE).

The output preserves the flat mesh's modified topology — including any
new vertices/polygons created by boolean cuts, polygon deletions, or
hand edits — while placing each vertex correctly on the curved surface.

This is exactly the workflow you want for UV-driven hole cuts:
  1. flatten_uv_to_geo.py — chair → flat
  2. <do whatever in flat space — boolean holes, delete polys, etc.>
  3. unflatten_uv_to_geo.py — flat → chair (with the holes now in 3D)

------------------------------------------------------------------------------
USAGE
------------------------------------------------------------------------------
Select TWO objects in this order, then run:
  1. The (modified) flat mesh — typically named "<NAME>_UV_FLAT"
  2. The original curved source mesh

OR select just the flat mesh — the script will try to find the source by
stripping "_UV_FLAT" from the flat object's name.

A new object "<FLAT_NAME>_BACKTO3D" is added to the scene with the flat
topology projected onto the curved source surface.

------------------------------------------------------------------------------
HOW IT WORKS
------------------------------------------------------------------------------
- Builds a UV → 3D triangle cache from the source curved mesh:
  every quad becomes two triangles, each storing its UV vertices and
  matching 3D vertex positions.
- Bins triangles into a 64×64 grid in UV space by their UV bbox, so
  per-vertex lookup is O(triangles-per-cell), not O(total triangles).
- For each vertex in the flat mesh:
    u, v = vert.x / SCALE, vert.z / SCALE
    find triangle whose UVs contain (u, v)
    compute barycentric weights of (u, v) within that UV triangle
    apply those weights to the 3D triangle vertices → curved 3D position
- Carries the flat mesh's UV tag and vertex maps over to the output.

Performance: ~1–3 seconds for typical 50k-vert meshes.

------------------------------------------------------------------------------
EDGE CASES
------------------------------------------------------------------------------
- Verts outside any UV island: kept at their flat position with a warning
  (so you can spot/fix them). Usually means the flat mesh extends beyond
  the original UV layout — expected if you padded the flat with extra geo.
- Non-overlapping UVs required. Mirrored/overlapping shells will resolve
  to the first matching tri, which may not be the intended side.
- SCALE must match the value used in the forward flatten step (default 1000).

------------------------------------------------------------------------------
LICENSE
------------------------------------------------------------------------------
MIT — free to use, modify, redistribute.

Author: Spenser Dickerson / SD Imaging  (sdimaging.art)
"""

import c4d
from c4d import documents
import time


# === UV overlap detection ====================================================

def detect_uv_overlap(src, grid_res=32, spread_frac=0.15):
    """Heuristically detect overlapping UV islands on the source mesh.

    Bins source polys into a UV grid by their UV centroid; for any cell with
    multiple polys, measures the 3D spread of those polys' world-space
    centroids. Large 3D spread inside a single UV cell ⇒ different mesh
    regions share the same UV space (typical of mirrored or stacked shells).

    Returns (n_overlap_cells, n_occupied_cells)."""
    uv_tag = src.GetTag(c4d.Tuvw)
    if not uv_tag:
        return (0, 0)
    src_pts = src.GetAllPoints()
    n_polys = src.GetPolygonCount()

    bbox_diag = src.GetRad().GetLength() * 2.0
    threshold = bbox_diag * spread_frac

    cells = [[[] for _ in range(grid_res)] for _ in range(grid_res)]
    for i in range(n_polys):
        poly = src.GetPolygon(i)
        uv = uv_tag.GetSlow(i)
        cu = (uv["a"].x + uv["b"].x + uv["c"].x + uv["d"].x) * 0.25
        cv = (uv["a"].y + uv["b"].y + uv["c"].y + uv["d"].y) * 0.25
        pa, pb, pc, pd = src_pts[poly.a], src_pts[poly.b], src_pts[poly.c], src_pts[poly.d]
        cx = (pa.x + pb.x + pc.x + pd.x) * 0.25
        cy = (pa.y + pb.y + pc.y + pd.y) * 0.25
        cz = (pa.z + pb.z + pc.z + pd.z) * 0.25
        gu = max(0, min(grid_res - 1, int(cu * grid_res)))
        gv = max(0, min(grid_res - 1, int(cv * grid_res)))
        cells[gv][gu].append((cx, cy, cz))

    overlap_cells = 0
    occupied = 0
    for gv in range(grid_res):
        for gu in range(grid_res):
            cell = cells[gv][gu]
            if len(cell) < 2:
                continue
            occupied += 1
            xs = [p[0] for p in cell]
            ys = [p[1] for p in cell]
            zs = [p[2] for p in cell]
            spread = max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs))
            if spread > threshold:
                overlap_cells += 1
    return (overlap_cells, occupied)


def warn_if_overlap(src):
    overlap, occupied = detect_uv_overlap(src)
    if overlap > 0:
        pct = (overlap / occupied * 100.0) if occupied else 0.0
        print(f"  ⚠ UV OVERLAP detected: {overlap}/{occupied} cells ({pct:.1f}%) "
              f"contain polys from disparate 3D regions.")
        print(f"     This typically means mirrored or stacked UV shells.")
        print(f"     Reverse pass: each flat vertex projects to whichever overlapping")
        print(f"                   tri matches first — likely wrong-side projection.")
        print(f"     Workaround: in UV editor, offset one half by 1 UV unit so")
        print(f"                 shells no longer overlap, then re-run.")
        print()


# === Configuration ===========================================================

# MUST match the SCALE used in flatten_uv_to_geo.py for the forward pass.
SCALE = 1000.0

# Spatial-grid resolution for UV → triangle lookup. 64 = 4096 cells.
# Higher = faster lookups, more memory. 32–128 is a reasonable range.
GRID_RES = 64

# =============================================================================


def build_uv_tri_cache(src):
    """Build a list of (uv0, uv1, uv2, p0, p1, p2) triangles from source mesh.
    Each quad emits two triangles (a-b-c, a-c-d). Returns triangles + a grid index."""
    uv_tag = src.GetTag(c4d.Tuvw)
    if not uv_tag:
        raise RuntimeError(f"'{src.GetName()}' has no UVW tag.")

    src_points = src.GetAllPoints()
    n_polys = src.GetPolygonCount()

    tris = []   # list of dicts: {uv0, uv1, uv2, p0, p1, p2, bbox}

    for i in range(n_polys):
        poly = src.GetPolygon(i)
        uv = uv_tag.GetSlow(i)

        ua, ub, uc, ud = uv["a"], uv["b"], uv["c"], uv["d"]
        pa, pb, pc, pd = src_points[poly.a], src_points[poly.b], src_points[poly.c], src_points[poly.d]
        is_tri = (poly.c == poly.d)

        # Triangle 1: a-b-c
        tris.append({
            "uv0": ua, "uv1": ub, "uv2": uc,
            "p0":  pa, "p1":  pb, "p2":  pc,
            "u_min": min(ua.x, ub.x, uc.x),
            "u_max": max(ua.x, ub.x, uc.x),
            "v_min": min(ua.y, ub.y, uc.y),
            "v_max": max(ua.y, ub.y, uc.y),
        })

        # Triangle 2: a-c-d (only for quads)
        if not is_tri:
            tris.append({
                "uv0": ua, "uv1": uc, "uv2": ud,
                "p0":  pa, "p1":  pc, "p2":  pd,
                "u_min": min(ua.x, uc.x, ud.x),
                "u_max": max(ua.x, uc.x, ud.x),
                "v_min": min(ua.y, uc.y, ud.y),
                "v_max": max(ua.y, uc.y, ud.y),
            })

    # Build spatial grid: each cell holds a list of triangle indices whose UV bbox overlaps
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
    """Compute barycentric weights of (u,v) within UV triangle uv0,uv1,uv2.
    Returns (w0, w1, w2) or None if degenerate."""
    den = ((uv1.y - uv2.y) * (uv0.x - uv2.x) +
           (uv2.x - uv1.x) * (uv0.y - uv2.y))
    if abs(den) < 1e-12:
        return None
    w0 = ((uv1.y - uv2.y) * (u - uv2.x) +
          (uv2.x - uv1.x) * (v - uv2.y)) / den
    w1 = ((uv2.y - uv0.y) * (u - uv2.x) +
          (uv0.x - uv2.x) * (v - uv2.y)) / den
    w2 = 1.0 - w0 - w1
    return (w0, w1, w2)


def find_curved_position(u, v, tris, grid):
    """Find the curved 3D position corresponding to UV (u,v).
    Returns Vector or None if no triangle contains (u,v)."""
    if not (0.0 <= u <= 1.0 and 0.0 <= v <= 1.0):
        return None
    cell_u = max(0, min(GRID_RES - 1, int(u * GRID_RES)))
    cell_v = max(0, min(GRID_RES - 1, int(v * GRID_RES)))

    # Test triangles in this cell first; widen if no match found
    for radius in (0, 1, 2):
        for dv in range(-radius, radius + 1):
            for du in range(-radius, radius + 1):
                if abs(du) != radius and abs(dv) != radius:
                    continue  # only test the ring at this radius
                cu, cv = cell_u + du, cell_v + dv
                if not (0 <= cu < GRID_RES and 0 <= cv < GRID_RES):
                    continue
                for ti in grid[cv][cu]:
                    t = tris[ti]
                    # Quick UV bbox reject
                    if u < t["u_min"] or u > t["u_max"] or v < t["v_min"] or v > t["v_max"]:
                        continue
                    bary = barycentric_2d(u, v, t["uv0"], t["uv1"], t["uv2"])
                    if bary is None:
                        continue
                    w0, w1, w2 = bary
                    eps = 1e-5
                    if w0 >= -eps and w1 >= -eps and w2 >= -eps:
                        # Inside (or on edge of) this triangle
                        return t["p0"] * w0 + t["p1"] * w1 + t["p2"] * w2
    return None


def main():
    doc = documents.GetActiveDocument()
    selected = doc.GetActiveObjects(c4d.GETACTIVEOBJECTFLAGS_NONE) or []

    flat = src = None

    if len(selected) >= 2 and selected[0].GetType() == c4d.Opolygon and selected[1].GetType() == c4d.Opolygon:
        flat, src = selected[0], selected[1]
    elif len(selected) == 1 and selected[0].GetType() == c4d.Opolygon:
        flat = selected[0]
        # Try to find source by stripping "_UV_FLAT" suffix
        name = flat.GetName()
        if name.endswith("_UV_FLAT"):
            target_name = name[:-len("_UV_FLAT")]
            o = doc.GetFirstObject()
            while o:
                if o.GetName() == target_name and o.GetType() == c4d.Opolygon:
                    src = o
                    break
                o = o.GetNext()

    if not flat or not src:
        c4d.gui.MessageDialog(
            "Select TWO polygon objects in this order:\n"
            "  1. The (modified) flat mesh — usually '*_UV_FLAT'\n"
            "  2. The original curved source mesh\n\n"
            "Or select just the flat mesh and the script will look\n"
            "for the source by stripping '_UV_FLAT' from the name."
        )
        return

    print("=" * 64)
    print("Flat UV Geometry → Curved 3D Mesh")
    print("=" * 64)
    print(f"Flat (input):    {flat.GetName()}  ({flat.GetPointCount()} verts, {flat.GetPolygonCount()} polys)")
    print(f"Source (curved): {src.GetName()}   ({src.GetPointCount()} verts, {src.GetPolygonCount()} polys)")
    print(f"SCALE: {SCALE}, GRID_RES: {GRID_RES}")

    # Pre-flight: warn about overlapping UV shells on source
    warn_if_overlap(src)

    t0 = time.time()

    # Build UV → 3D lookup from source
    tris, grid = build_uv_tri_cache(src)
    t_cache = time.time()
    print(f"  Built UV tri cache: {len(tris)} tris in {t_cache - t0:.2f}s")

    # Project each flat vertex back to curved 3D
    flat_pts = flat.GetAllPoints()
    n_flat = len(flat_pts)

    new_pts = [None] * n_flat
    n_outside = 0
    for i, pt in enumerate(flat_pts):
        u = pt.x / SCALE
        v = pt.z / SCALE
        curved = find_curved_position(u, v, tris, grid)
        if curved is None:
            new_pts[i] = pt   # leave at flat position; flag for reporting
            n_outside += 1
        else:
            new_pts[i] = curved

    t_project = time.time()
    print(f"  Projected {n_flat} verts in {t_project - t_cache:.2f}s")
    if n_outside > 0:
        print(f"  ⚠ {n_outside} verts had no UV match — left at flat position.")
        print(f"     (Typically means flat geo extends beyond original UV islands.)")

    # Build output PolygonObject with same topology as flat
    n_polys_flat = flat.GetPolygonCount()
    out = c4d.PolygonObject(n_flat, n_polys_flat)
    out.SetAllPoints(new_pts)
    for i in range(n_polys_flat):
        out.SetPolygon(i, flat.GetPolygon(i))
    out.Message(c4d.MSG_UPDATE)
    out.SetName(flat.GetName() + "_BACKTO3D")
    out.MakeTag(c4d.Tphong)
    doc.InsertObject(out)
    c4d.EventAdd()

    # Carry over UV tag from flat (preserves any UV edits done in flat-space)
    flat_uv = flat.GetTag(c4d.Tuvw)
    if flat_uv:
        try:
            new_uv = c4d.UVWTag(n_polys_flat)
            out.InsertTag(new_uv)
            for i in range(n_polys_flat):
                uv = flat_uv.GetSlow(i)
                new_uv.SetSlow(i, uv["a"], uv["b"], uv["c"], uv["d"])
            print(f"  UV tag carried over: {n_polys_flat} faces.")
        except Exception as e:
            print(f"  ⚠ UV tag transfer failed: {e}")

    # Carry over vertex maps from flat
    t = flat.GetFirstTag()
    n_vmap = 0
    while t:
        if t.GetType() == c4d.Tvertexmap:
            try:
                src_data = t.GetAllHighlevelData()
                if src_data and len(src_data) == n_flat:
                    new_vmap = c4d.VariableTag(c4d.Tvertexmap, n_flat)
                    new_vmap.SetName(t.GetName())
                    out.InsertTag(new_vmap)
                    new_vmap.SetAllHighlevelData(src_data)
                    n_vmap += 1
            except Exception as e:
                print(f"  ⚠ Vertex map '{t.GetName()}' transfer failed: {e}")
        t = t.GetNext()
    if n_vmap:
        print(f"  Vertex maps carried over: {n_vmap}")

    c4d.EventAdd()

    t_done = time.time()
    print(f"  Wrote object in {t_done - t_project:.2f}s")
    print(f"  TOTAL: {t_done - t0:.2f}s")
    print()
    print(f"Result: {out.GetName()}")
    print(f"  Verts: {n_flat}, Polys: {n_polys_flat}")
    print(f"  Bbox half-extent: {out.GetRad()}")
    print()
    print("If the result has unwanted gaps/seams along UV island edges, run a")
    print("'Optimize' command on it (Mesh → Commands → Optimize) to weld coincident")
    print("verts that share 3D positions across UV seams.")
    print("=" * 64)


if __name__ == "__main__":
    main()
