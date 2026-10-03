"""Reading the camera videos: metadata, real frame timestamps, and frame-aligned reading of 3 cameras.

Security camera (NVR) exports often have a nominal fps that is not the real one and a variable
frame rate, so the timestamps are read from the file instead of trusting the header.
"""
import json
import shutil
import subprocess
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class VideoInfo:
    path: str
    width: int
    height: int
    codec: str
    fps_header: float            # what the container claims
    frame_count: int             # frames actually readable (timestamps) or the header count
    duration_s: float
    fps_measured: float = None   # 1 / median frame interval
    vfr: bool = None             # variable frame rate
    max_gap_s: float = None      # longest interval between two frames (dropped frames)

    @property
    def size(self):
        return (self.width, self.height)

    def summary(self):
        return {
            "path": self.path, "size": [self.width, self.height], "codec": self.codec,
            "fps_header": round(self.fps_header, 3),
            "fps_measured": None if self.fps_measured is None else round(self.fps_measured, 3),
            "frames": self.frame_count, "duration_s": round(self.duration_s, 3),
            "vfr": self.vfr, "max_gap_s": None if self.max_gap_s is None else round(self.max_gap_s, 4),
        }


def open_video(path):
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        raise IOError(f"cannot open video {path}")
    return cap


def frame_timestamps(path):
    """Presentation time (s) of every frame, sorted. ffprobe if installed (fast, no decoding), else OpenCV."""
    if shutil.which("ffprobe"):
        out = subprocess.run(
            ["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries", "packet=pts_time",
             "-of", "json", path], capture_output=True, text=True, check=True).stdout
        ts = [float(p["pts_time"]) for p in json.loads(out).get("packets", []) if "pts_time" in p]
        if ts:
            return np.sort(np.array(ts))
    cap = open_video(path)
    ts = []
    while cap.grab():
        ts.append(cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0)
    cap.release()
    return np.sort(np.array(ts))


def timing_stats(ts, tolerance=0.25):
    """(fps_measured, vfr, max_gap_s) from frame timestamps.

    vfr: more than 1% of the frame intervals differ from the median interval by more than `tolerance`.
    """
    if len(ts) < 3:
        return None, None, None
    dt = np.diff(ts)
    dt = dt[dt > 0]
    if dt.size == 0:
        return None, None, None
    med = float(np.median(dt))
    off = np.abs(dt - med) > tolerance * med
    return 1.0 / med, bool(off.mean() > 0.01), float(dt.max())


def probe_video(path, timestamps=True):
    cap = open_video(path)
    fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
    codec = "".join(chr((fourcc >> 8 * i) & 0xFF) for i in range(4)).strip("\x00 ") or "?"
    info = VideoInfo(
        path=path,
        width=int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
        height=int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
        codec=codec,
        fps_header=float(cap.get(cv2.CAP_PROP_FPS) or 0.0),
        frame_count=int(cap.get(cv2.CAP_PROP_FRAME_COUNT)),
        duration_s=0.0,
    )
    cap.release()
    if info.fps_header > 0:
        info.duration_s = info.frame_count / info.fps_header
    if timestamps:
        ts = frame_timestamps(path)
        if len(ts):
            info.frame_count = len(ts)
            info.fps_measured, info.vfr, info.max_gap_s = timing_stats(ts)
            if info.fps_measured:
                info.duration_s = float(ts[-1] - ts[0]) + 1.0 / info.fps_measured
    return info


def read_frames(path, indices):
    """The frames at the given indices (seeking), as a list of BGR images (None if unreadable)."""
    cap = open_video(path)
    frames = []
    for i in indices:
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(i))
        ok, frame = cap.read()
        frames.append(frame if ok else None)
    cap.release()
    return frames


def apply_masks(frame, rects):
    """Black out [x, y, w, h] boxes (OSD timestamps); returns the same array."""
    for x, y, w, h in rects or []:
        frame[int(y):int(y + h), int(x):int(x + w)] = 0
    return frame


class SyncedReader:
    """Iterates the cameras' videos together: yields (frame_index, {cam: BGR frame}).

    The recordings are hardware synced, so frame i of every camera is the same instant once the
    per-camera start offsets are dropped. Stops at the end of the shortest video.
    """

    def __init__(self, paths, offsets=None, masks=None, step=1):
        self.paths = dict(paths)
        self.offsets = {cam: int((offsets or {}).get(cam, 0)) for cam in self.paths}
        self.masks = masks or {}
        self.step = max(1, int(step))

    def __iter__(self):
        caps = {cam: open_video(p) for cam, p in self.paths.items()}
        try:
            for cam, cap in caps.items():
                for _ in range(self.offsets[cam]):
                    if not cap.grab():
                        return
            i = 0
            while True:
                frames = {}
                for cam, cap in caps.items():
                    if i % self.step == 0:
                        ok, frame = cap.read()
                    else:
                        ok, frame = cap.grab(), None
                    if not ok:
                        return
                    if frame is not None:
                        frames[cam] = apply_masks(frame, self.masks.get(cam))
                if frames:
                    yield i, frames
                i += 1
        finally:
            for cap in caps.values():
                cap.release()
