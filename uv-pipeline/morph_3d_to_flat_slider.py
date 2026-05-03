"""
3D ↔ Flat UV Morph Slider  (split-topology, live)
==================================================

A Cinema 4D Python script that turns any polygon mesh with a UVW tag into
a live 3D ↔ flat-UV morph driven by a single 0-1 slider.

Drag the **Factor** slider:
- **Factor=0**: looks identical to the original 3D mesh (split vertices coincide
  on their source 3D positions — no visible seams).
- **Factor=1**: fully flat UV unwrap with proper UV-island separation
  (each polygon-vertex pair lands at its own UV coord; seams visibly split).
- **Factor in between**: smooth morph; the seams visibly fan apart as the
  islands separate. Looks like the head "explodes" along its UV seams.

Topology is split throughout (one output vertex per polygon-vertex corner,
e.g. ~4664 verts for a 1168-vertex head). At every factor value the mesh is
the SAME topology — only positions change. So you can keyframe the slider,
animate it, render it, all without topology changes.

------------------------------------------------------------------------------
USAGE
------------------------------------------------------------------------------
1. Select the polygon mesh you want to morph in the Object Manager.
   Must have a UVW tag. (Use "Current State to Object" first if the mesh
   comes from a generator.)
2. Extensions → Script Manager → Load Script File → load this file → Execute.
3. A new object  "<NAME>_UV_MORPH_SPLIT"  is added with:
   - Split-topology mesh (4× the polygon corner count)
   - "Factor" slider (0-1) and "Scale" slider (0-200) in User Data
   - "UV Morph SPLIT" Python tag that does the per-vertex morph live
4. Drag the Factor slider on the new object to morph between 3D and flat.

------------------------------------------------------------------------------
HOW IT WORKS
------------------------------------------------------------------------------
Setup (one-time, on script execution):
  - Walk every polygon corner of the source mesh
  - For each corner, allocate a new output vertex with:
      - source_vertex_idx (the original 3D vertex it came from)
      - uv_at_corner (the UV coordinate at this exact corner)
  - Build the output mesh: same poly count, but with split vertex indices
    (4 unique verts per quad, 3 per triangle)
  - Initial positions = source 3D positions (so factor=0 looks like the
    original head — split verts coincide perfectly)
  - Cache (orig_3d_pos, uv_at_corner) per output vertex in the BaseContainer
  - Add Factor + Scale UD sliders
  - Insert a Python tag with embedded morph logic

Runtime (every frame, in the Python tag):
  - Read cached orig_3d_pos + uv per output vertex
  - flat_pos = (uv.x * scale, -uv.y * scale, 0)
  - new_pos = orig_3d_pos + (flat_pos - orig_3d_pos) * factor
  - SetAllPoints

Why this is better than averaging UV-per-source-vertex:
  - At factor=1 you get a TRUE UV-island split (real seams, proper
    flat layout matching what uvtomesh in Scene Nodes produces)
  - At factor=0 the split verts coincide on source positions, so the
    mesh visually looks identical to the welded original
  - In between, the seams smoothly fan apart — no UV averaging blur

------------------------------------------------------------------------------
LIMITATIONS
------------------------------------------------------------------------------
- Output vertex count is ~4× higher than source (one per polygon-vertex)
- The output mesh has split topology even at factor=0; if you need to use
  it for downstream operations that care about welded topology, use a
  Connect/Optimize step or pick from the source mesh instead
- Recomputing the cache requires re-running the script (e.g., after editing
  the source mesh's topology)
"""

import c4d


SCALE_DEFAULT = 50.0
SCALE_MAX = 200.0
CACHE_SLOT = 99999
CACHE_OFFSET_UV = 10000


def build_split_topology(src):
    """
    Build a split-topology mesh from src.

    For each (polygon, corner) in src, allocate a new output vertex.
    Returns: (out_vert_count, out_polys, src_vert_for_out, uv_for_out)
      - out_vert_count: total output vertex count
      - out_polys:      list of (a, b, c, d) tuples (output vertex indices per poly)
      - src_vert_for_out: list[i] = source vertex idx that output vertex i came from
      - uv_for_out:     list[i] = UV vector at output vertex i's source corner
    """
    src_uvtag = src.GetTag(c4d.Tuvw)
    if src_uvtag is None:
        raise RuntimeError("Source mesh has no UVW tag")

    out_vert_count = 0
    out_polys = []
    src_vert_for_out = []
    uv_for_out = []

    for poly_idx in range(src.GetPolygonCount()):
        poly = src.GetPolygon(poly_idx)
        uv = src_uvtag.GetSlow(poly_idx)
        src_corners = [poly.a, poly.b, poly.c, poly.d]
        uv_corners = [uv["a"], uv["b"], uv["c"], uv["d"]]
        is_tri = (poly.c == poly.d)

        if is_tri:
            a_out = out_vert_count
            b_out = out_vert_count + 1
            c_out = out_vert_count + 2
            d_out = c_out  # tri: d == c
            for ci in range(3):
                src_vert_for_out.append(src_corners[ci])
                uv_for_out.append(uv_corners[ci])
            out_vert_count += 3
            out_polys.append((a_out, b_out, c_out, d_out))
        else:
            a_out = out_vert_count
            b_out = out_vert_count + 1
            c_out = out_vert_count + 2
            d_out = out_vert_count + 3
            for ci in range(4):
                src_vert_for_out.append(src_corners[ci])
                uv_for_out.append(uv_corners[ci])
            out_vert_count += 4
            out_polys.append((a_out, b_out, c_out, d_out))

    return out_vert_count, out_polys, src_vert_for_out, uv_for_out


def add_slider_ud(obj, name, short_name, vmin, vmax, default, step):
    """Add a Float user-data slider; returns its DescID."""
    bc_ud = c4d.GetCustomDataTypeDefault(c4d.DTYPE_REAL)
    bc_ud[c4d.DESC_NAME] = name
    bc_ud[c4d.DESC_SHORT_NAME] = short_name
    bc_ud[c4d.DESC_MIN] = vmin
    bc_ud[c4d.DESC_MAX] = vmax
    bc_ud[c4d.DESC_STEP] = step
    bc_ud[c4d.DESC_DEFAULT] = default
    bc_ud[c4d.DESC_CUSTOMGUI] = c4d.CUSTOMGUI_REALSLIDER
    bc_ud[c4d.DESC_ANIMATE] = c4d.DESC_ANIMATE_ON
    bc_ud[c4d.DESC_UNIT] = c4d.DESC_UNIT_FLOAT
    new_id = obj.AddUserData(bc_ud)
    obj[new_id] = default
    return new_id


def make_python_tag_code(factor_id, scale_id):
    """Generate the Python tag code with the actual UD DescIDs baked in."""
    fid = factor_id
    sid = scale_id
    return f'''import c4d

def main():
    obj = op.GetObject()
    bc = obj.GetDataInstance()
    cache = bc.GetContainer({CACHE_SLOT})
    if not cache:
        return
    fid = c4d.DescID(c4d.DescLevel({fid[0].id}, {fid[0].dtype}, {fid[0].creator}),
                    c4d.DescLevel({fid[1].id}, {fid[1].dtype}, {fid[1].creator}))
    sid = c4d.DescID(c4d.DescLevel({sid[0].id}, {sid[0].dtype}, {sid[0].creator}),
                    c4d.DescLevel({sid[1].id}, {sid[1].dtype}, {sid[1].creator}))
    factor = obj[fid] if obj[fid] is not None else 0.0
    scale  = obj[sid] if obj[sid] is not None else {SCALE_DEFAULT}
    n = obj.GetPointCount()
    new_pts = []
    for i in range(n):
        orig = cache.GetVector(i)
        uv = cache.GetVector({CACHE_OFFSET_UV} + i)
        flat = c4d.Vector(uv.x * scale, -uv.y * scale, 0)
        new_pts.append(orig + (flat - orig) * factor)
    obj.SetAllPoints(new_pts)
    # NOTE: do NOT call obj.Message(c4d.MSG_UPDATE) here — would cause
    # infinite recursive evaluation (Message → tag re-fires → SetAllPoints → Message …)
'''


def main():
    src = doc.GetActiveObject()
    if src is None:
        c4d.gui.MessageDialog("Select a polygon mesh first.")
        return
    if src.GetType() != c4d.Opolygon:
        c4d.gui.MessageDialog(
            "Selected object must be a polygon mesh.\n"
            "Run 'Current State to Object' first if it's from a generator."
        )
        return
    if src.GetTag(c4d.Tuvw) is None:
        c4d.gui.MessageDialog("Selected mesh has no UVW tag — cannot build a UV morph.")
        return

    doc.StartUndo()

    # 1. Build split topology
    out_n, out_polys, src_vert_for_out, uv_for_out = build_split_topology(src)

    # 2. Create the new mesh + populate
    morph_obj = c4d.PolygonObject(out_n, len(out_polys))
    morph_obj.SetName(src.GetName() + "_UV_MORPH_SPLIT")

    # Initial positions = source 3D positions per output vertex
    src_pts = [src.GetPoint(i) for i in range(src.GetPointCount())]
    init_pts = [src_pts[src_vert_for_out[i]] for i in range(out_n)]
    morph_obj.SetAllPoints(init_pts)

    # Polygons
    for pi, (a, b, c, d) in enumerate(out_polys):
        morph_obj.SetPolygon(pi, c4d.CPolygon(a, b, c, d))
    morph_obj.Message(c4d.MSG_UPDATE)

    # Position offset (next to the source)
    rad = src.GetRad()
    morph_obj.SetAbsPos(src.GetAbsPos() + c4d.Vector(rad.x * 3, 0, 0))

    # Insert Phong tag for nice display
    morph_obj.InsertTag(c4d.BaseTag(c4d.Tphong))

    doc.InsertObject(morph_obj, pred=src)
    doc.AddUndo(c4d.UNDOTYPE_NEW, morph_obj)

    # 3. Cache (orig_3d, uv) per output vertex
    data_bc = c4d.BaseContainer()
    for i in range(out_n):
        orig = src_pts[src_vert_for_out[i]]
        uv = uv_for_out[i]
        data_bc.SetVector(i, orig)
        data_bc.SetVector(CACHE_OFFSET_UV + i, uv)
    morph_obj.GetDataInstance().SetContainer(CACHE_SLOT, data_bc)

    # 4. Add UD sliders
    factor_id = add_slider_ud(
        morph_obj, "Factor", "Factor",
        0.0, 1.0, 0.0, 0.01,
    )
    scale_id = add_slider_ud(
        morph_obj, "Scale", "Scale",
        0.0, SCALE_MAX, SCALE_DEFAULT, 1.0,
    )

    # 5. Python tag
    py_tag = c4d.BaseTag(c4d.Tpython)
    py_tag.SetName("UV Morph SPLIT")
    morph_obj.InsertTag(py_tag)
    py_tag[c4d.TPYTHON_CODE] = make_python_tag_code(factor_id, scale_id)
    doc.AddUndo(c4d.UNDOTYPE_NEW, py_tag)

    doc.SetActiveObject(morph_obj, c4d.SELECTION_NEW)
    doc.EndUndo()
    c4d.EventAdd()

    print(f"[UV Morph SPLIT] Created '{morph_obj.GetName()}'")
    print(f"[UV Morph SPLIT]   Source: {src.GetPointCount()} verts, {src.GetPolygonCount()} polys")
    print(f"[UV Morph SPLIT]   Output: {out_n} verts (split), {len(out_polys)} polys")
    print(f"[UV Morph SPLIT]   Drag the 'Factor' slider on the new object 0 → 1 to morph")


if __name__ == "__main__":
    main()
