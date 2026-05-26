"""Cross-platform compute detection for the reconstruction pipeline.

The pipeline runs on Windows/Linux with an NVIDIA CUDA GPU and on Apple-Silicon Macs.
The dense stage is **OpenMVS** in both cases: it uses CUDA when an NVIDIA GPU is present
(fast) and falls back to CPU on Mac. COLMAP does sparse SfM on CPU on all platforms.

This module only *detects and reports* the plan — it shells out to no binaries beyond an
optional `nvidia-smi` probe — so it is safe to import and test anywhere.
"""
from __future__ import annotations
import os
import platform
import shutil
import subprocess
from dataclasses import dataclass, field


def _has_nvidia_gpu() -> bool:
    """True if an NVIDIA CUDA GPU appears usable (nvidia-smi present and runs)."""
    if shutil.which("nvidia-smi") is None:
        return False
    try:
        subprocess.run(["nvidia-smi"], capture_output=True, timeout=5, check=True)
        return True
    except Exception:
        return False


@dataclass
class ComputeConfig:
    os: str
    machine: str
    has_cuda: bool
    sfm_backend: str = "colmap"          # COLMAP SfM is CPU + cross-platform
    dense_backend: str = "openmvs"       # OpenMVS dense on every platform
    openmvs_use_cuda: bool = False       # OpenMVS CUDA densify when an NVIDIA GPU is present
    threads: int = 0
    notes: list[str] = field(default_factory=list)


def detect_compute(force_cuda: bool | None = None) -> ComputeConfig:
    """Detect OS/GPU and return the recommended reconstruction backend plan.

    `force_cuda` overrides detection (useful for tests / CI). The dense backend is always
    OpenMVS; only whether it uses CUDA varies by machine.
    """
    os_name = platform.system()
    machine = platform.machine()
    has_cuda = _has_nvidia_gpu() if force_cuda is None else bool(force_cuda)
    threads = os.cpu_count() or 1

    notes: list[str] = []
    if has_cuda:
        notes.append("NVIDIA CUDA GPU detected -> OpenMVS dense uses CUDA (fast).")
    elif os_name == "Darwin":
        notes.append("Apple Silicon / no CUDA -> OpenMVS dense runs on CPU "
                     "(works, but dense MVS is slow on large sets).")
    else:
        notes.append("No CUDA GPU -> OpenMVS dense runs on CPU (slow on large sets).")
    notes.append("COLMAP performs sparse SfM on CPU on all platforms.")

    return ComputeConfig(os=os_name, machine=machine, has_cuda=has_cuda,
                         openmvs_use_cuda=has_cuda, threads=threads, notes=notes)


def doctor() -> str:
    """Human-readable summary of the detected compute plan (for a `photrecon doctor` cmd)."""
    c = detect_compute()
    lines = [f"OS: {c.os} ({c.machine})   CUDA GPU: {'yes' if c.has_cuda else 'no'}   "
             f"threads: {c.threads}",
             f"SfM: {c.sfm_backend} (CPU)   Dense: {c.dense_backend} "
             f"({'CUDA' if c.openmvs_use_cuda else 'CPU'})"]
    lines += [f"  - {n}" for n in c.notes]
    return "\n".join(lines)


if __name__ == "__main__":
    print(doctor())
