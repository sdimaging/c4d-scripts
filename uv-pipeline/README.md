# UV Pipeline

Three standalone Cinema 4D Python scripts that together let you do 2D operations on a curved 3D mesh by working in its UV layout.

## The big idea

Cutting holes / adding details / running boolean operations on a curved organic surface is hard. C4D booleans struggle with curved tangencies; voxel-based methods round everything; sculpting destroys topology.

But on a **flat 2D plane**, all of these operations are trivial.

This pipeline gives you the trick: flatten the mesh into UV space → do the 2D op → wrap the result back onto the curved surface using the same UV correspondence.

```
flatten_uv_to_geo.py             unflatten_uv_to_geo.py
─────────────────────            ──────────────────────
        ┌─────────────┐                  ┌─────────────┐
chair  →│   FLAT      │  ── modify ──→  │ FLAT WITH   │ → holed chair
(3D)    │  PLANAR GEO │   (booleans,    │   HOLES     │   (3D)
        │  in UV space│    deletes,     └─────────────┘
        └─────────────┘    selects)            │
                                                ▼
                                     barycentric UV→3D projection
                                     into the original curved surface
```

## Scripts

### 1. `flatten_uv_to_geo.py` — chair → flat

Drops a planar polygon mesh next to your source. Each input poly is reproduced at its UV coords on the Y=0 plane (X = U × SCALE, Z = V × SCALE). Vertex deduplication keeps connected polys welded; UV seams stay open.

- Carries UVs over (so Surface Deformer / Pose Morph can map back later)
- Carries vertex maps over (weights remapped across UV-seam splits)
- Detects overlapping UV islands at startup and warns
- Runtime: ~0.2s on a 48k-poly mesh

**Usage:** select your source mesh → run.

### 2. `unflatten_uv_to_geo.py` — modified flat → chair

Reverse of #1. Takes the flat mesh (with whatever modifications you've made — boolean cuts, polygon deletions, even new geometry) and projects every vertex back onto the curved 3D source via barycentric UV→3D mapping.

Handles new vertices created by booleans correctly: as long as a new vertex's UV-equivalent position falls within an island, it gets a correct curved 3D position via barycentric weights.

**Usage:** select the modified flat + the original curved source → run. (Or just the flat — it will look up the source by name suffix.)

**Caveats:**
- Output sits on the linear cage of the source polys, not the SDS-smoothed surface. Apply SDS to the result for a smooth final, or use a denser source mesh.
- Mirrored / overlapping UV shells produce wrong-side-projection. Detected and warned at startup.

### 3. `place_children_on_curved.py` — children of flat → on curved

For every decoration object parented under the flat mesh (cylinders, spheres, hardware, custom geo), produce a copy at the corresponding spot on the curved surface, oriented to the surface normal.

Four modes via `MODE` config at top:

- `"instance"` *(default)* — output `c4d.Oinstance` objects in Render Instance mode. **LIVE link to the source.** Edit the master cylinder (slide an axis null, change geometry, swap to Connect+Cloner) and all hundreds of placed copies update automatically.
- `"place"` — baked independent clones. Each placement is editable separately.
- `"deform"` — vertex-level deform (children must be poly meshes); mesh wraps surface curvature.
- `"all"` — produces all three output groups for comparison.

Handles the "frozen-coord poly" case automatically: when you Disconnect/Split a consolidated mesh in C4D, all pieces inherit a shared axis but have unique vertex data. The script uses bbox center (`Mg * Mp`) for placement and re-centers vertex data on clone, so post-Split pieces work without manually running "Center Axis" on each.

**Hierarchy convention:** Null objects under the flat are treated as transparent organizing wrappers (script descends into them). Non-Null objects are placement candidates. So a hierarchy like `Flat → Group_Null → Inst_001, Inst_002, ...` produces N placed instances, not 1 placed group.

## Workflow example: cut hole pattern into a chair

1. **Build chair** with a clean UV unwrap (4-6 flat non-overlapping islands)
2. **Run `flatten_uv_to_geo.py`** on the chair → get `chair_UV_FLAT`
3. **Build cutter master** (e.g. a cylinder with an Axis Offset null inside)
4. **Distribute cutter instances** across the flat (Cloner → bake to children, or X-Particles → cache, or hand-place)
5. **Parent the cutters under `chair_UV_FLAT`** in the OM
6. **Run `place_children_on_curved.py`** with `MODE = "instance"` → cutters appear on the curved chair, oriented to the surface normals
7. **Volume Builder + Subtract** the curved cutter group from the chair → final chair with holes
8. **Tweak the master cutter** (slide its axis offset, change geometry) → all 100s of cutter instances on the curved chair update live → re-bake the Volume Builder → done

The whole pipeline is procedural and re-runnable.

## Configuration

All three scripts share `SCALE = 1000.0` at the top. UV coords (0–1) are mapped to scene units by × SCALE. Default 1000 makes a flat mesh that's roughly the same size as a typical product (1m). Adjust if your scene is in metres or mm.

If you change SCALE in one script, change it in all three so the pipeline round-trips correctly.

## License

MIT. See repo-root [LICENSE](../LICENSE).
