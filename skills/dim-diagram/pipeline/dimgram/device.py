"""Cross-platform compute-device selection.

Works on Windows/Linux with an NVIDIA CUDA GPU, on Apple Silicon Macs via Metal (MPS),
and falls back to CPU everywhere. Only the optional SAM path needs torch, so torch is
imported lazily and its absence is handled gracefully.
"""
from __future__ import annotations
import platform


def select_torch_device(prefer: str | None = None) -> str:
    """Return the best available torch device string: 'cuda', 'mps', or 'cpu'.

    `prefer` (e.g. "cpu") forces a choice when valid. CUDA wins when present (the
    Windows/Linux NVIDIA case); MPS is used on Apple Silicon; otherwise CPU.
    """
    if prefer in ("cuda", "mps", "cpu"):
        return prefer
    try:
        import torch
        if torch.cuda.is_available():
            return "cuda"
        mps = getattr(torch.backends, "mps", None)
        if mps is not None and mps.is_available():
            return "mps"
    except Exception:
        pass
    return "cpu"


def device_info() -> dict:
    """Diagnostics for logging / the `dimgram doctor`-style report."""
    info = {"os": platform.system(), "machine": platform.machine(),
            "torch": None, "device": "cpu", "cuda": False, "mps": False}
    try:
        import torch
        info["torch"] = torch.__version__
        info["cuda"] = bool(torch.cuda.is_available())
        mps = getattr(torch.backends, "mps", None)
        info["mps"] = bool(mps is not None and mps.is_available())
    except Exception:
        pass
    info["device"] = select_torch_device()
    return info
