"""Pluggable detector backends for the object pose estimator.

The default ``color`` backend segments objects by RGB ratio rules and
estimates pose from point statistics — lightweight, dependency-free, and
real-time on CPU. The ``onnx`` backend is the integration point for a
deep-learning detector (e.g. a YOLO/DETR model exported to ONNX): it shares
the same Detection output contract, so swapping backends does not touch the
rest of the pipeline.
"""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Detection:
    """A single detected object in world coordinates."""

    name: str
    position: tuple  # (x, y, z) world frame, meters
    yaw: float  # principal-axis yaw around world z, radians
    size: tuple  # axis-aligned extents (dx, dy, dz), meters
    num_points: int


class ColorSegmentationBackend:
    """RGB-ratio segmentation + centroid/PCA pose estimation."""

    name = "color"

    # name -> (mask function over r, g, b int16 arrays)
    # Tuned against the ogre2-rendered scene: PBR lighting lifts the green
    # channel of the blue block to ~0.75*b, so the blue rule uses an
    # absolute b-g margin instead of a strict ratio.
    TARGETS = {
        "sample_block": lambda r, g, b: (b > 80) & (r < 0.5 * b) & (b > g + 25),
        "sample_vial_red": lambda r, g, b: (r > 1.4 * g) & (r > 1.4 * b) & (r > 60),
    }

    def target_names(self):
        return list(self.TARGETS.keys())

    def detect(self, world_points, r, g, b, min_points):
        detections = []
        for name, mask_fn in self.TARGETS.items():
            mask = mask_fn(r, g, b)
            pts = world_points[mask]
            if pts.shape[0] < min_points:
                continue

            centroid = pts.mean(axis=0)
            xy = pts[:, :2] - centroid[:2]
            # Principal axis in the horizontal plane -> yaw estimate.
            cov = np.cov(xy.T) if xy.shape[0] > 2 else np.eye(2)
            eigvals, eigvecs = np.linalg.eigh(cov)
            principal = eigvecs[:, int(np.argmax(eigvals))]
            yaw = float(np.arctan2(principal[1], principal[0]))

            mins, maxs = pts.min(axis=0), pts.max(axis=0)
            size = tuple((maxs - mins).tolist())
            # The camera only sees the top surface; report the volumetric
            # center assuming the object rests on the bench below it.
            detections.append(
                Detection(
                    name=name,
                    position=(
                        float(centroid[0]),
                        float(centroid[1]),
                        float((maxs[2] + mins[2]) / 2.0),
                    ),
                    yaw=yaw,
                    size=size,
                    num_points=int(pts.shape[0]),
                )
            )
        return detections


class OnnxDetectorBackend:
    """Deep-learning detector integration point (ONNX Runtime).

    Contract: implement ``detect`` with the same signature/return type as
    ColorSegmentationBackend. A typical implementation projects the RGB
    image through the model for 2D boxes/masks, then looks up the box pixels
    in the organized cloud for 3D pose. Requires ``onnxruntime`` and a
    trained model; both are deployment-specific, so this backend refuses to
    start without an explicit model path.
    """

    name = "onnx"

    def __init__(self, model_path: str):
        if not model_path:
            raise ValueError(
                "detector_backend:=onnx requires onnx_model_path to be set; "
                "falling back to the color backend is the supported default."
            )
        try:
            import onnxruntime  # noqa: F401
        except ImportError as ex:
            raise RuntimeError(
                "onnxruntime is not installed; pip install onnxruntime"
            ) from ex
        self._model_path = model_path
        raise NotImplementedError(
            "Wire your exported detection model here: load the ONNX session, "
            "run it on the RGB image, and convert masks to Detection objects."
        )

    def target_names(self):
        return []

    def detect(self, world_points, r, g, b, min_points):
        return []


def make_backend(kind: str, onnx_model_path: str = "", logger=None):
    """Factory: choose a detector backend by name."""
    if kind == "onnx":
        return OnnxDetectorBackend(onnx_model_path)
    if kind != "color" and logger is not None:
        logger.warn(f"unknown detector_backend '{kind}', using 'color'")
    return ColorSegmentationBackend()
