"""The three YAML files in mocap/config/ and the paths they point to (relative to mocap/)."""
import os

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG_DIR = os.path.join(ROOT, "config")


def load(name, directory=None):
    """load("rig") -> dict from config/rig.yaml."""
    with open(os.path.join(directory or CONFIG_DIR, name + ".yaml")) as fh:
        return yaml.safe_load(fh)


def path(rel):
    """A rig.yaml path made absolute against mocap/ (absolute paths pass through)."""
    return rel if os.path.isabs(rel) else os.path.join(ROOT, rel)


def camera_paths(pattern, cameras):
    """{cam: absolute video path} for a "{cam}" pattern such as data/calib/{cam} (extension optional)."""
    out = {}
    for cam in cameras:
        p = path(pattern.format(cam=cam))
        out[cam] = p if os.path.splitext(p)[1].lower() in VIDEO_EXT else find_video(*os.path.split(p))
    return out


def session_paths(rig, session):
    """{cam: path} of one mopping recording, data/sessions/<session>/<cam>.*"""
    folder = os.path.join(path(rig["sessions_dir"]), session)
    return {cam: find_video(folder, cam) for cam in rig["cameras"]}


VIDEO_EXT = (".mp4", ".avi", ".mkv", ".mov", ".dav", ".h264", ".h265", ".ts")


def find_video(folder, stem):
    """folder/stem.<any video extension>; the .mp4 name is returned (and reported missing) if none exists."""
    for ext in VIDEO_EXT:
        candidate = os.path.join(folder, stem + ext)
        if os.path.isfile(candidate):
            return candidate
    return os.path.join(folder, stem + ".mp4")
