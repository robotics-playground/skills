# 3D Measurement, Validation, and Export

Once the mesh is registered to the bundle's metric frame, measure it, cross-check it against the
2D dimensions, and export.

## Measurements

- **Overall L × W × H:** the **oriented bounding box** of the aligned mesh
  (`open3d` `get_oriented_bounding_box`, or `trimesh` `bounding_box_oriented`). Because the mesh
  is already canonically aligned (from registration), the OBB axes correspond to the dim-diagram
  front/top/side directions.
- **Features (holes, bosses, faces):** fit primitives to the mesh with RANSAC —
  `open3d` `segment_plane` for faces, cylinder/sphere fits (`pyransac3d`, PyMeshLab) for hole
  diameters and fillets. Report point-to-point distances by picking mesh vertices.
- **Uncertainty:** combine the fit residuals with the bundle's scale uncertainty (`scale_sigma`).

## Validation against the bundle (the headline trust signal)

Compare each mesh measurement to the corresponding `dim-diagram` 2D dimension and report a
`ValidationReport`:

```
pairs: [(dim_name, mesh_mm, bundle_mm, delta_pct), ...]
max_delta_pct, notes
```

A small `delta_pct` across the board means the independent 2D and 3D measurements agree —
strong evidence the model is metrically sound. A large delta on one axis points to a capture or
registration problem on that axis.

## Export

- **Mesh:** OBJ / PLY / STL via `trimesh` or `open3d`.
- **STEP (best-effort):** convert via FreeCAD / `pythonocc-core` for CAD use. Mesh→STEP is
  lossy/approximate; mark it optional and warn the user it's a tessellated solid, not a
  feature-based CAD model.
- **JSON:** the measurement set + the validation report.

## References
- Open3D OBB & RANSAC: http://www.open3d.org/docs/release/
- PyMeshLab: https://pymeshlab.readthedocs.io/
