"""
UV Islands → Flat Polygon Geometry  (with Vertex Map transfer)
==============================================================

A Cinema 4D Python script that flattens a polygon mesh into planar geometry
laid out in UV space. Each input polygon is reproduced at its UV coordinates
on the Y=0 plane (X=U×SCALE, Z=V×SCALE), with vertex deduplication preserving
UV-island connectivity.

Also transfers any Vertex Map tags from source → flat output, with weights
correctly remapped across UV-seam splits. So if you painted a vertex map
on the curved chair (e.g. to mark hole regions), the same painted regions
are immediately available on the flat copy to drive boolean / selection /
field-based operations.

Useful for any workflow that benefits from doing 2D operations on flat geo
(boolean cutouts, painted hole maps, polygon-selection by image, etc.) then
deforming the modified flat geo back onto the original curved mesh via Surface
Deformer or Pose Morph.

------------------------------------------------------------------------------
USAGE
------------------------------------------------------------------------------
1. Select your target polygon mesh in the Object Manager.
2. Extensions → Script Manager → Load Script File → load this file → Execute.
3. A new object  "<NAME>_UV_FLAT"  is added to the scene.

If your target is a Subdivision Surface or other generator output, use
"Current State to Object" first to bake it into an editable polygon mesh
with a UVW tag.

------------------------------------------------------------------------------
WHEN THIS IS USEFUL
------------------------------------------------------------------------------
- You painted a hole map / detail map in UV space and want to use it to
  drive geometric cutouts on the actual mesh.
- You want to perform precise 2D boolean operations and re-project the
  result back onto a curved surface.
- You need a debug visualization of your UV layout as actual geometry.
- Any "flatten → modify → wrap back" pipeline.

------------------------------------------------------------------------------
REQUIREMENTS
------------------------------------------------------------------------------
- Cinema 4D 2026 (probably works back to R20; untested)
- A polygon object with a UVW tag
- UV islands should NOT overlap. Overlapping shells will dedup vertices
  that aren't actually topologically connected, creating welded seams.
  (Mirrored unwraps are the common cause — flatten one side at a time.)

------------------------------------------------------------------------------
WRAPPING THE FLAT GEO BACK TO 3D
------------------------------------------------------------------------------
Two approaches once you've cut/modified the flat geo:

  Option A — Surface Deformer
    Add a Surface Deformer to the flat object, bind to the original curved
    mesh. Surface Deformer projects each flat point onto the closest point
    of the bind target. Works fine if the flat geo retains its UV tag (this
    script copies UVs over by default).

  Option B — Pose Morph
    Add a Pose Morph tag to the flat object. Set base pose = current flat
    positions. Add a morph target whose positions = the original curved
    mesh's points (same topology / index correspondence assumed). Slider
    blends flat ↔ curved, even after polygon deletion / hole cuts, because
    morphs map by vertex index.

------------------------------------------------------------------------------
HOW IT WORKS
------------------------------------------------------------------------------
- Iterates every polygon in the source; reads per-poly UVs via UVWTag.GetSlow.
- Each unique UV position becomes one vertex in the output (dedup keyed by
  quantized UV, default 6 decimal places). Polygons connected in UV space
  share the resulting vertex; UV seams stay open.
- Triangles are detected via (poly.c == poly.d) and the 4th index is
  collapsed.
- A new UVWTag is constructed via the explicit c4d.UVWTag(count) ctor and
  populated with copies of the source's UVs, so the flat geo is wrap-ready.
- For each Vertex Map tag on the source, a parallel tag is created on the
  flat geo. Each new flat vertex is tagged with the source vertex it
  originated from; the source weight is then copied across, so UV-seam
  splits inherit the same weight on both sides.

Performance: ~0.2 seconds for a 48k-poly mesh on a modern CPU.

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
        print(f"     Forward pass: vertices may be falsely welded across overlap.")
        print(f"     Workaround: in UV editor, offset one half by 1 UV unit so")
        print(f"                 shells no longer overlap, then re-run.")
        print()


# === Configuration ===========================================================

# UV space (0–1) is mapped to scene units by multiplying by SCALE.
# Default 1000 makes the flat layout 1000×1000 units — comfortable to work
# with at typical scene scales. Adjust if your scene is in metres or mm.
SCALE = 1000.0

# Vertex dedup precision. Two UV vertices closer than 1/QUANT in either axis
# are merged. 1,000,000 = 6-decimal precision; safe for any sane UV unwrap.
QUANT = 1000000

# =============================================================================


def main():
    doc = documents.GetActiveDocument()
    src = doc.GetActiveObject()

    if not src or src.GetType() != c4d.Opolygon:
        c4d.gui.MessageDialog(
            "Select a polygon mesh first.\n\n"
            "If your target is a Subdivision Surface or other generator output,\n"
            "use 'Current State to Object' first to bake to editable poly."
        )
        return

    uv_tag = src.GetTag(c4d.Tuvw)
    if not uv_tag:
        c4d.gui.MessageDialog(f"'{src.GetName()}' has no UVW tag.")
        return

    n_polys = src.GetPolygonCount()

    print("=" * 64)
    print("UV Islands → Flat Polygon Geometry")
    print("=" * 64)
    print(f"Source: {src.GetName()}")
    print(f"  Polygons: {n_polys}")
    print(f"  Points:   {src.GetPointCount()}")

    # Pre-flight: warn about overlapping UV shells
    warn_if_overlap(src)

    t0 = time.time()

    # Cache hot method references — meaningful speed-up in the inner loop
    GetPolygon = src.GetPolygon
    GetSlow    = uv_tag.GetSlow
    Vector     = c4d.Vector

    vert_dict = {}
    new_pts = []
    new_polys_data = []      # list of (a, b, c, d) index tuples
    src_vert_for_flat = []   # parallel to new_pts: which source vertex each flat vert came from

    for i in range(n_polys):
        src_poly = GetPolygon(i)
        uv = GetSlow(i)
        ua, ub, uc, ud = uv["a"], uv["b"], uv["c"], uv["d"]

        # Dedup vertex A by quantized UV position
        ka = (int(ua.x * QUANT + 0.5), int(ua.y * QUANT + 0.5))
        a = vert_dict.get(ka)
        if a is None:
            a = len(new_pts)
            new_pts.append(Vector(ua.x * SCALE, 0.0, ua.y * SCALE))
            src_vert_for_flat.append(src_poly.a)
            vert_dict[ka] = a

        kb = (int(ub.x * QUANT + 0.5), int(ub.y * QUANT + 0.5))
        b = vert_dict.get(kb)
        if b is None:
            b = len(new_pts)
            new_pts.append(Vector(ub.x * SCALE, 0.0, ub.y * SCALE))
            src_vert_for_flat.append(src_poly.b)
            vert_dict[kb] = b

        kc = (int(uc.x * QUANT + 0.5), int(uc.y * QUANT + 0.5))
        c = vert_dict.get(kc)
        if c is None:
            c = len(new_pts)
            new_pts.append(Vector(uc.x * SCALE, 0.0, uc.y * SCALE))
            src_vert_for_flat.append(src_poly.c)
            vert_dict[kc] = c

        if src_poly.c == src_poly.d:
            d = c   # triangle: collapse 4th index onto 3rd
        else:
            kd = (int(ud.x * QUANT + 0.5), int(ud.y * QUANT + 0.5))
            d = vert_dict.get(kd)
            if d is None:
                d = len(new_pts)
                new_pts.append(Vector(ud.x * SCALE, 0.0, ud.y * SCALE))
                src_vert_for_flat.append(src_poly.d)
                vert_dict[kd] = d

        new_polys_data.append((a, b, c, d))

    t_geo = time.time()
    print(f"  Built geometry in {t_geo - t0:.2f}s "
          f"— {len(new_pts)} verts, {len(new_polys_data)} polys")

    # Construct the output polygon object
    out = c4d.PolygonObject(len(new_pts), len(new_polys_data))
    out.SetAllPoints(new_pts)
    for i, (a, b, c, d) in enumerate(new_polys_data):
        out.SetPolygon(i, c4d.CPolygon(a, b, c, d))
    out.Message(c4d.MSG_UPDATE)
    out.SetName(src.GetName() + "_UV_FLAT")

    # Phong + insert FIRST so the object lands even if UV-tag copy fails later
    out.MakeTag(c4d.Tphong)
    doc.InsertObject(out)
    c4d.EventAdd()

    # Copy UVs onto the flat geo — needed for Surface Deformer / shading workflows
    try:
        flat_uv_tag = c4d.UVWTag(n_polys)
        out.InsertTag(flat_uv_tag)
        for i in range(n_polys):
            uv = uv_tag.GetSlow(i)
            flat_uv_tag.SetSlow(i, uv["a"], uv["b"], uv["c"], uv["d"])
        c4d.EventAdd()
        print(f"  UV tag copied: {n_polys} faces.")
    except Exception as e:
        print(f"  ⚠ UV tag copy skipped: {e}")
        print(f"  (Object still created. UVs optional if wrapping via Pose Morph.)")

    # Copy ALL Vertex Map tags from source — weights remapped via src_vert_for_flat[]
    # so UV-seam-split verts inherit the same weight on both sides of the seam.
    src_vmap_tags = []
    t = src.GetFirstTag()
    while t:
        if t.GetType() == c4d.Tvertexmap:
            src_vmap_tags.append(t)
        t = t.GetNext()

    n_flat = len(new_pts)
    for src_vmap in src_vmap_tags:
        try:
            src_data = src_vmap.GetAllHighlevelData()
            if src_data is None or len(src_data) == 0:
                print(f"  ⚠ Vertex map '{src_vmap.GetName()}' has no data — skipped.")
                continue

            # Build remapped weights: flat_weights[i] = src_weights[src_vert_for_flat[i]]
            new_data = [src_data[src_vert_for_flat[i]] for i in range(n_flat)]

            new_vmap = c4d.VariableTag(c4d.Tvertexmap, n_flat)
            new_vmap.SetName(src_vmap.GetName())
            out.InsertTag(new_vmap)
            new_vmap.SetAllHighlevelData(new_data)
            c4d.EventAdd()
            print(f"  Vertex map copied: '{src_vmap.GetName()}' "
                  f"({len(src_data)} src verts → {n_flat} flat verts)")
        except Exception as e:
            print(f"  ⚠ Vertex map '{src_vmap.GetName()}' transfer failed: {e}")

    if not src_vmap_tags:
        print("  (No vertex maps on source — nothing to transfer.)")

    t_done = time.time()
    print(f"  Wrote object in {t_done - t_geo:.2f}s")
    print(f"  TOTAL: {t_done - t0:.2f}s")
    print()
    print(f"Result: {out.GetName()}")
    print(f"  Lives on Y=0 plane, X=U×{SCALE}, Z=V×{SCALE}")
    print(f"  Source bbox half-extent: {src.GetRad()}")
    print(f"  Flat   bbox half-extent: {out.GetRad()}")
    print()
    print("Next steps:")
    print("  1. Modify the flat geo (boolean cuts, vertex map → polygon delete, etc.)")
    print("  2. Wrap back to the curved mesh via:")
    print("       - Surface Deformer bound to the original curved object, OR")
    print("       - Pose Morph with curved positions as a target.")
    print("=" * 64)


if __name__ == "__main__":
    main()
