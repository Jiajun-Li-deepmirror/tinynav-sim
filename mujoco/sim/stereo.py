"""Retinify stereo depth on onnxruntime -- the engine behind contract_face.

Port of reference/tinynav/core/models_trt.py::RetinifyTRT (the TensorRT
flavour production perception runs): same neural net -- the fleet's retinify
0.1.5 dynamic export, carried verbatim in assets/stereo/ -- same calling
contract (left/right mono8 at native size, FLOAT NHWC (1, H, W, 1),
disparity-only output), but on onnxruntime so there is no per-arch/TRT-.plan
rebuild to run the sim on a new machine.

`disparity_to_depth` is verbatim from the same reference file. Depth here is
float32 metres, invalid (disparity <= 0) = 0; the wire format (uint16 mm,
mono16) is contract_face's business.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np

ONNX = Path(__file__).resolve().parent.parent / "assets" / "stereo" / \
    "retinify_0_1_5_dynamic.onnx"


def disparity_to_depth(disparity: np.ndarray, baseline: float,
                       focal_length: float) -> np.ndarray:
    """Port of reference/tinynav/core/models_trt.py::disparity_to_depth."""
    disparity = np.asarray(disparity, dtype=np.float32)
    baseline = float(np.asarray(baseline).reshape(-1)[0])
    focal_length = float(np.asarray(focal_length).reshape(-1)[0])

    if baseline <= 0.0:
        raise ValueError(f"baseline must be positive, got {baseline}")
    if focal_length <= 0.0:
        raise ValueError(f"focal_length must be positive, got {focal_length}")

    depth = np.zeros_like(disparity, dtype=np.float32)
    valid = np.isfinite(disparity) & (disparity > 0.0)
    depth[valid] = (baseline * focal_length) / disparity[valid]
    return depth


class RetinifyStereo:
    """Drop-in depth source: infer(left, right, baseline, fx) -> (disp, depth_m).

    CUDA first; a CPU-only runtime is a constructor-time failure, not a silent
    100x slowdown mid-run.
    """

    def __init__(self, onnx_path: Path = ONNX) -> None:
        import onnxruntime as ort

        so = ort.SessionOptions()
        so.log_severity_level = 3
        self._sess = ort.InferenceSession(
            str(onnx_path), so,
            providers=["CUDAExecutionProvider", "CPUExecutionProvider"])
        if self._sess.get_providers()[0] != "CUDAExecutionProvider":
            raise RuntimeError(
                "onnxruntime CUDAExecutionProvider unavailable -- install "
                "onnxruntime-gpu with its CUDA 13 libs (see docker/mujoco.Dockerfile)")
        self._out = self._sess.get_outputs()[0].name

    def infer(self, left: np.ndarray, right: np.ndarray,
              baseline: float, focal_length: float) -> tuple:
        h, w = left.shape[:2]
        lt = left.astype(np.float32, copy=False)[None, :, :, None]
        rt = right.astype(np.float32, copy=False)[None, :, :, None]
        disp = self._sess.run([self._out], {"left": lt, "right": rt}
                              )[0][0, :, :, 0].astype(np.float32)
        return disp, disparity_to_depth(disp, baseline, focal_length)
