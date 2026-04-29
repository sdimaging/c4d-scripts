# c4d-scripts

A growing collection of standalone Python scripts for **Cinema 4D 2024+**. Each script is self-contained — drop it into C4D's Script Manager (Extensions → Script Manager → Load Script File), run, done. No external dependencies, no plugins to install.

These complement (but don't require) the Cinema 4D MCP server at [sdimaging/cinema4d-mcp](https://github.com/sdimaging/cinema4d-mcp).

---

## Collections

### [`uv-pipeline/`](uv-pipeline/) — Flatten / unflatten / wrap children

A three-script pipeline for performing 2D operations on a 3D mesh by working in UV space:

1. **`flatten_uv_to_geo.py`** — flatten a polygon mesh into planar geometry laid out in UV space (X = U × scale, Z = V × scale). Vertex maps and UVs preserved.
2. **`unflatten_uv_to_geo.py`** — reverse: project the (modified) flat mesh back onto the curved 3D source via barycentric UV→3D mapping. Handles new vertices created by booleans / topology changes.
3. **`place_children_on_curved.py`** — for each decoration object parented under the flat mesh, produce a copy on the curved 3D surface. Three modes: live `c4d.Oinstance` linkage, baked rigid clone, or vertex-level deform.

Both directions detect overlapping UV islands at startup and warn before producing wrong output. Works on any mesh with a clean UV layout.

Use cases:
- Cut hole patterns into a curved chair / sculpture by working on its flat UV layout (boolean cuts, manual delete-polys, Painter-driven masks)
- Stamp hardware (studs, screws, cylinders) on a curved surface using flat-space placement
- Wrap signage / decals onto organic geometry

See [`uv-pipeline/README.md`](uv-pipeline/README.md) for the full workflow walkthrough.

---

## Why standalone scripts?

C4D's Script Manager is the fastest path from idea → working result for a single artist on a single machine:

- Zero install (just save the .py and run)
- Lives in the Script Manager menu permanently
- Easy to fork / modify per project
- No auth, no tokens, no MCP, no Claude needed

The companion [Cinema 4D MCP server](https://github.com/sdimaging/cinema4d-mcp) wraps similar capabilities as AI-callable tools — better for Claude-driven workflows. Standalone scripts are better when you just want to use them yourself.

---

## Contributing / forking

MIT licensed. Fork freely. If you build on top of these and want to upstream, PRs welcome.

---

## License

MIT. © 2026 Spenser Dickerson / SD Imaging. See [LICENSE](LICENSE).
