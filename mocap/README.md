# mocap — 3-camera pose capture of a person mopping

Three synced fisheye security cameras (room corners) → 2D body keypoints per camera (YOLO26-pose)
→ 3D keypoints in a floor-based world frame (metres, Z up). The robot that shadows the motion is
a separate later step; it only reads the 3D output.

Videos never go to git: they live in `mocap/data/` on the local PC (ignored).

## Setup
```bash
python3 -m venv mocap/venv
mocap/venv/bin/pip install -r mocap/requirements.txt
cd mocap && venv/bin/python -m pytest -q tests
```

## Data layout
```
mocap/data/calib/cam0.mp4 cam1.mp4 cam2.mp4             ChArUco video per camera (intrinsics)
mocap/data/static_board/cam0.mp4 cam1.mp4 cam2.mp4      board lying still, seen by all 3 (extrinsics)
mocap/data/sessions/<name>/cam0.mp4 cam1.mp4 cam2.mp4   mopping recordings
```
Any common extension works (.mp4 .avi .mkv .mov .dav). Camera names and paths: `config/rig.yaml`.
Board sizes: `config/board.yaml` (fill in the real printed sizes).

## Commands (run inside `mocap/`)
| command | does |
|---|---|
| `python -m mocap probe` | resolution, real fps, variable frame rate, dropped frames, sync and calib-vs-recording resolution checks; contact sheets in `out/probe/` |
