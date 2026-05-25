# Reconstruction with COLMAP (SfM + MVS)

The reconstruction engine is **COLMAP** via `pycolmap`. A GPU (CUDA) is strongly recommended for
dense MVS; CPU works but is slow. Ship a Dockerfile (COLMAP + CUDA) for reproducible installs.

## Stages

1. **Feature extraction & matching (SfM front-end).**
   ```python
   import pycolmap
   pycolmap.extract_features(database_path, image_dir)         # SIFT by default
   pycolmap.match_exhaustive(database_path)                    # small sets; else vocab-tree/sequential
   ```
   Seed the camera model from the bundle's `camera.yaml` intrinsics when available — a known
   focal length stabilises the reconstruction.

2. **Mapping (sparse).**
   ```python
   maps = pycolmap.incremental_mapping(database_path, image_dir, sparse_dir)
   ```
   Produces camera **poses** + a **sparse point cloud**. Inspect reprojection error; drop images
   that fail to register.

3. **Dense (MVS).** PatchMatch stereo + fusion:
   ```python
   pycolmap.patch_match_stereo(dense_dir)       # GPU
   pycolmap.stereo_fusion(dense_dir / "fused.ply", dense_dir)
   ```

4. **Meshing + cleanup.** Poisson (COLMAP `poisson_mesher` or Open3D
   `create_from_point_cloud_poisson`), then PyMeshLab: remove floaters/non-manifold faces,
   decimate, fill small holes.

## Masking (turntable)

When the object rotates against a static background, mask the object in each frame so SfM keys
on the object, not the room. Reuse `dim-diagram`'s `SamSegmenter` (MobileSAM) or a color key.
COLMAP reads masks alongside images to ignore background features.

## Pitfalls

- **Texture-poor / reflective regions** → sparse or noisy points; handled downstream by
  silhouette carving (`bundle-and-registration.md`).
- **Insufficient overlap** → drift / failed registration; shoot more, at multiple elevations.
- **Up-to-scale only** → COLMAP geometry has no absolute scale; that's why registration to the
  dim-diagram bundle (next reference) is mandatory for metric output.

## References
- COLMAP: https://colmap.github.io/  ·  pycolmap: https://github.com/colmap/pycolmap
- Open3D: http://www.open3d.org/docs/release/  ·  PyMeshLab: https://pymeshlab.readthedocs.io/
