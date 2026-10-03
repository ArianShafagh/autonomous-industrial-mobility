"""`python -m mocap probe`: what are the videos, really? Run this first on new recordings.

Per video: resolution, codec, header vs measured fps, frame count, variable frame rate, dropped
frames. Per synced group (the 3 cameras of one recording): do they agree? Per camera: is the
calibration video the same resolution as the recordings (intrinsics are only valid then)?
Also writes a contact sheet per video to check field of view, board size and OSD overlays.
"""
import os

import cv2
import numpy as np
import yaml

from mocap import config
from mocap.video import probe_video, read_frames

FPS_TOL = 0.01          # synced cameras: measured fps may differ by 1%
DROP_FACTOR = 2.5       # an interval this many times the nominal one means dropped frames


def check_video(info):
    """Problems of a single video, as readable strings."""
    issues = []
    if info.fps_measured and info.fps_header and abs(info.fps_measured - info.fps_header) > 0.05 * info.fps_header:
        issues.append(f"header says {info.fps_header:.2f} fps but frames come at {info.fps_measured:.2f} fps")
    if info.vfr:
        issues.append("variable frame rate: frames must be re-timed by their timestamps")
    if info.max_gap_s and info.fps_measured and info.max_gap_s > DROP_FACTOR / info.fps_measured:
        issues.append(f"dropped frames: longest gap {info.max_gap_s:.3f}s")
    return issues


def check_group(infos):
    """Problems across the cameras of one synced recording ({cam: VideoInfo})."""
    issues = []
    fps = {cam: i.fps_measured or i.fps_header for cam, i in infos.items()}
    ref = np.median(list(fps.values()))
    for cam, f in fps.items():
        if ref and abs(f - ref) > FPS_TOL * ref:
            issues.append(f"{cam} runs at {f:.2f} fps, the others at {ref:.2f}")
    counts = {cam: i.frame_count for cam, i in infos.items()}
    if max(counts.values()) - min(counts.values()) > 1:
        issues.append(f"frame counts differ {counts} - check sync / set frame_offsets")
    return issues


def check_resolution(calib_info, other_infos, cam):
    """Calibration video vs recordings of the same camera: sizes must match."""
    return [f"{cam}: {label} is {i.width}x{i.height} but the calibration video is "
            f"{calib_info.width}x{calib_info.height} - intrinsics would not apply"
            for label, i in other_infos.items() if i.size != calib_info.size]


def contact_sheet(path, n=6, cols=3, tile_width=640, masks=None, label=""):
    """n evenly spaced frames tiled into one image; OSD masks outlined in red."""
    cap = cv2.VideoCapture(path)
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = cap.get(cv2.CAP_PROP_FPS) or 0
    cap.release()
    idx = np.linspace(0, max(total - 1, 0), n).astype(int)
    tiles = []
    for i, frame in zip(idx, read_frames(path, idx)):
        if frame is None:
            continue
        for x, y, w, h in masks or []:
            cv2.rectangle(frame, (int(x), int(y)), (int(x + w), int(y + h)), (0, 0, 255), 3)
        scale = tile_width / frame.shape[1]
        tile = cv2.resize(frame, (tile_width, int(round(frame.shape[0] * scale))))
        text = f"{label} frame {i}" + (f"  t={i / fps:.1f}s" if fps else "")
        cv2.putText(tile, text, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4)
        cv2.putText(tile, text, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        tiles.append(tile)
    if not tiles:
        return None
    h = max(t.shape[0] for t in tiles)
    tiles = [cv2.copyMakeBorder(t, 0, h - t.shape[0], 0, 0, cv2.BORDER_CONSTANT) for t in tiles]
    while len(tiles) % cols:
        tiles.append(np.zeros_like(tiles[0]))
    rows = [np.hstack(tiles[r:r + cols]) for r in range(0, len(tiles), cols)]
    return np.vstack(rows)


def rig_groups(rig):
    """{group: {cam: path}} for every recording the rig config knows about and that exists."""
    cams = rig["cameras"]
    groups = {"calib": config.camera_paths(rig["calib_videos"], cams),
              "static_board": config.camera_paths(rig["static_board"], cams)}
    sessions = config.path(rig["sessions_dir"])
    if os.path.isdir(sessions):
        for name in sorted(os.listdir(sessions)):
            if os.path.isdir(os.path.join(sessions, name)):
                groups["session/" + name] = config.session_paths(rig, name)
    return {g: {c: p for c, p in paths.items() if os.path.isfile(p)} for g, paths in groups.items()}


def explicit_groups(args):
    """Paths given on the command line: a folder is one group (its videos), files form one group."""
    groups, files = {}, {}
    for a in args:
        if os.path.isdir(a):
            vids = sorted(f for f in os.listdir(a) if os.path.splitext(f)[1].lower() in config.VIDEO_EXT)
            groups[os.path.basename(os.path.normpath(a))] = {
                os.path.splitext(f)[0]: os.path.join(a, f) for f in vids}
        else:
            files[os.path.splitext(os.path.basename(a))[0]] = a
    if files:
        groups["files"] = files
    return groups


def run(paths=None, timestamps=True, sheets=True, out_dir=None):
    rig = config.load("rig")
    groups = explicit_groups(paths) if paths else rig_groups(rig)
    out_dir = out_dir or os.path.join(config.path(rig["out_dir"]), "probe")
    os.makedirs(out_dir, exist_ok=True)
    report, problems = {}, []
    infos = {}
    for group, cams in groups.items():
        if not cams:
            print(f"[{group}] no videos found")
            continue
        print(f"[{group}]")
        infos[group] = {}
        for cam, path in cams.items():
            info = probe_video(path, timestamps=timestamps)
            infos[group][cam] = info
            s = info.summary()
            fps = s["fps_measured"] if s["fps_measured"] is not None else s["fps_header"]
            print(f"  {cam:8s} {info.width}x{info.height} {info.codec:5s} {fps:7.2f} fps "
                  f"{info.frame_count:7d} frames {info.duration_s:8.1f} s  vfr={info.vfr}  {os.path.basename(path)}")
            issues = check_video(info)
            problems += [f"[{group}] {cam}: {m}" for m in issues]
            report.setdefault(group, {})[cam] = dict(s, issues=issues)
            if sheets:
                img = contact_sheet(path, masks=rig.get("osd_masks", {}).get(cam), label=cam)
                if img is not None:
                    sheet = os.path.join(out_dir, group.replace("/", "_") + f"_{cam}.jpg")
                    cv2.imwrite(sheet, img)
        if group != "calib" and len(cams) > 1:
            problems += [f"[{group}] {m}" for m in check_group(infos[group])]
    for cam, calib_info in infos.get("calib", {}).items():
        others = {g: i[cam] for g, i in infos.items() if g != "calib" and cam in i}
        problems += check_resolution(calib_info, others, cam)

    report["problems"] = problems
    with open(os.path.join(out_dir, "probe.yaml"), "w") as fh:
        yaml.safe_dump(report, fh, sort_keys=False)
    print("\nproblems:" if problems else "\nno problems found")
    for p in problems:
        print("  - " + p)
    print(f"report and contact sheets: {out_dir}")
    return report
