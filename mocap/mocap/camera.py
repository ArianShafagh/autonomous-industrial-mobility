"""One calibrated camera: lens model (intrinsics) + pose in the world (extrinsics).

The lens model decides which OpenCV functions project and undistort, so every stage goes through
this class instead of calling cv2 directly:
  pinhole  - k1 k2 p1 p2 k3            (cv2.projectPoints / cv2.undistortPoints)
  rational - k1..k6 p1 p2 (8 coefs)    (same functions, wide-angle lenses)
  fisheye  - Kannala-Brandt k1..k4     (cv2.fisheye.*)
World frame: metres, Z up, origin on the floor (set by the extrinsic calibration).
R, t map world -> camera:  X_cam = R @ X_world + t.
"""
from dataclasses import dataclass, field

import cv2
import numpy as np
import yaml

MODELS = ("pinhole", "rational", "fisheye")


@dataclass
class Camera:
    name: str
    model: str
    K: np.ndarray                     # 3x3
    dist: np.ndarray                  # distortion coefficients of `model`
    size: tuple                       # (width, height) in pixels the calibration is valid for
    R: np.ndarray = field(default_factory=lambda: np.eye(3))
    t: np.ndarray = field(default_factory=lambda: np.zeros(3))
    rms: float = None                 # intrinsic calibration RMS reprojection error, px

    def __post_init__(self):
        if self.model not in MODELS:
            raise ValueError(f"{self.name}: unknown lens model '{self.model}', expected one of {MODELS}")
        self.K = np.asarray(self.K, dtype=np.float64).reshape(3, 3)
        self.dist = np.asarray(self.dist, dtype=np.float64).ravel()
        self.R = np.asarray(self.R, dtype=np.float64).reshape(3, 3)
        self.t = np.asarray(self.t, dtype=np.float64).ravel()
        self.size = tuple(int(v) for v in self.size)
        if self.model == "fisheye" and self.dist.size != 4:
            raise ValueError(f"{self.name}: fisheye needs 4 distortion coefficients, got {self.dist.size}")

    @property
    def center(self):
        """Camera position in the world, metres."""
        return -self.R.T @ self.t

    @property
    def Rt(self):
        """3x4 [R|t]: the projection matrix for undistorted, normalised image coordinates."""
        return np.hstack([self.R, self.t[:, None]])

    def project(self, X):
        """(N, 3) world points -> (N, 2) distorted pixel coordinates (NaN rows stay NaN)."""
        X = np.asarray(X, dtype=np.float64).reshape(-1, 3)
        out = np.full((len(X), 2), np.nan)
        ok = np.isfinite(X).all(axis=1)
        if ok.any():
            rvec, _ = cv2.Rodrigues(self.R)
            if self.model == "fisheye":
                uv, _ = cv2.fisheye.projectPoints(X[ok].reshape(-1, 1, 3), rvec, self.t, self.K, self.dist)
            else:
                uv, _ = cv2.projectPoints(X[ok], rvec, self.t, self.K, self.dist)
            out[ok] = uv.reshape(-1, 2)
        return out

    def undistort(self, uv):
        """(N, 2) distorted pixels -> (N, 2) normalised coordinates (x/z, y/z in the camera frame)."""
        uv = np.asarray(uv, dtype=np.float64).reshape(-1, 2)
        out = np.full_like(uv, np.nan)
        ok = np.isfinite(uv).all(axis=1)
        if ok.any():
            pts = uv[ok].reshape(-1, 1, 2)
            if self.model == "fisheye":
                xy = cv2.fisheye.undistortPoints(pts, self.K, self.dist)
            else:
                xy = cv2.undistortPoints(pts, self.K, self.dist)
            out[ok] = xy.reshape(-1, 2)
        return out

    def to_dict(self):
        return {
            "name": self.name, "model": self.model, "image_size": list(self.size),
            "K": self.K.tolist(), "dist": self.dist.tolist(),
            "R": self.R.tolist(), "t": self.t.tolist(),
            "rms_px": None if self.rms is None else float(self.rms),
        }

    @classmethod
    def from_dict(cls, d):
        return cls(name=d["name"], model=d["model"], K=d["K"], dist=d["dist"], size=d["image_size"],
                   R=d.get("R", np.eye(3)), t=d.get("t", np.zeros(3)), rms=d.get("rms_px"))

    def save(self, path):
        with open(path, "w") as fh:
            yaml.safe_dump(self.to_dict(), fh, sort_keys=False)

    @classmethod
    def load(cls, path):
        with open(path) as fh:
            return cls.from_dict(yaml.safe_load(fh))
