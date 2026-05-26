# Reconstruction: COLMAP SfM + OpenMVS dense (cross-platform)

Sparse Structure-from-Motion is **COLMAP** (`pycolmap`); dense reconstruction + meshing is
**OpenMVS**. This split is what makes the pipeline run on both a CUDA Windows/Linux PC and an
Apple-Silicon Mac — COLMAP's own dense MVS (`patch_match_stereo`) is CUDA-only and is **not**
used.

## Why this split (the cross-platform core)

| Stage | Engine | Windows/Linux + NVIDIA | macOS (Apple Silicon) |
|-------|--------|------------------------|------------------------|
| Sparse SfM (poses) | COLMAP / pycolmap | CPU | CPU |
| Dense + mesh | **OpenMVS** | **CUDA (fast)** | **CPU (works, slower)** |

`photrecon.platform.detect_compute()` probes the OS and `nvidia-smi`, then sets
`openmvs_use_cuda`. There is **one dense code path**; only the CUDA/CPU flag differs. Inspect the
plan anytime:

```bash
python -m photrecon.platform
# OS: Darwin (arm64)   CUDA GPU: no   threads: 8
# SfM: colmap (CPU)   Dense: openmvs (CPU)
```

## 1. Sparse SfM (COLMAP / pycolmap) — CPU, all platforms

```python
import pycolmap
pycolmap.extract_features(database_path, image_dir)     # SIFT
pycolmap.match_exhaustive(database_path)                # small sets; else vocab-tree/sequential
maps = pycolmap.incremental_mapping(database_path, image_dir, sparse_dir)
```

Seed the camera model from the bundle's `camera.yaml` intrinsics when available — a known focal
length stabilises the reconstruction. Output: camera **poses** + a **sparse point cloud**.

## 2. Dense + mesh (OpenMVS) — CUDA or CPU

Export the COLMAP result to OpenMVS, then run its stages. The binaries are the same on every
platform; the CUDA build accelerates `DensifyPointCloud`:

```bash
InterfaceCOLMAP   -i sparse/ -o scene.mvs
DensifyPointCloud scene.mvs            # CUDA build -> GPU; CPU build -> CPU (Mac)
ReconstructMesh   scene_dense.mvs
RefineMesh        scene_dense_mesh.mvs
TextureMesh       scene_dense_mesh_refine.mvs   # optional texture
```

`platform.py` chooses which OpenMVS build/flag to invoke. On Mac, `DensifyPointCloud` runs on
CPU — fine for modest sets, slow for hundreds of high-res images (decimate / cap resolution).

Then clean the mesh with **PyMeshLab** (remove floaters, decimate, fill small holes) and hand it
to `register`.

## Masking (turntable)

When the object rotates against a static background, mask the object per frame so SfM keys on the
object, not the room. Reuse `dim-diagram`'s `SamSegmenter` (MobileSAM — itself auto-selects
CUDA/MPS/CPU) or a colour key. COLMAP and OpenMVS both accept per-image masks.

## Install

- **CUDA (Windows via WSL2 / Linux):** `docker/Dockerfile.cuda` (OpenMVS built
  `-DOpenMVS_USE_CUDA=ON`), run with `--gpus all`.
- **CPU (Linux/CI):** `docker/Dockerfile.cpu`.
- **macOS:** native install (Homebrew/conda + build OpenMVS) — Docker Desktop can't pass through
  Metal, so containers give no acceleration; run OpenMVS on CPU on the host.

## Pitfalls

- **Texture-poor / reflective regions** → sparse/noisy points; handled downstream by silhouette
  carving (`bundle-and-registration.md`).
- **Insufficient overlap** → drift / failed registration; shoot more, at multiple elevations.
- **Up-to-scale only** → COLMAP+OpenMVS geometry has no absolute scale; registration to the
  dim-diagram bundle supplies metric scale + canonical axes.
- **Large sets on Mac CPU** → dense is slow; cap image count/resolution or borrow a CUDA box.

## References
- COLMAP / pycolmap: https://colmap.github.io/ · https://github.com/colmap/pycolmap
- OpenMVS: https://github.com/cdcseacave/openMVS
- Open3D / PyMeshLab: http://www.open3d.org/docs/release/ · https://pymeshlab.readthedocs.io/
