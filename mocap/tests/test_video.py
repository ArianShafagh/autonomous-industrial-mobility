"""Video metadata, frame timing and frame-aligned reading of several cameras."""
import numpy as np

from conftest import frame_number
from mocap.probe import check_group, check_resolution, check_video, contact_sheet
from mocap.video import SyncedReader, VideoInfo, probe_video, timing_stats


def test_probe_reads_size_fps_and_frames(make_video):
    info = probe_video(make_video("a.avi", 40, size=(320, 240), fps=20.0))
    assert info.size == (320, 240)
    assert info.frame_count == 40
    assert abs(info.fps_header - 20.0) < 1e-6
    assert abs(info.fps_measured - 20.0) < 0.1
    assert info.vfr is False
    assert check_video(info) == []


def test_timing_detects_variable_rate_and_drops():
    steady = np.arange(100) / 25.0
    assert timing_stats(steady)[1] is False
    jittery = np.cumsum(np.tile([0.03, 0.05], 50))
    assert timing_stats(jittery)[1] is True
    dropped = np.concatenate([np.arange(50), np.arange(55, 100)]) / 25.0
    fps, vfr, gap = timing_stats(dropped)
    assert abs(fps - 25.0) < 1e-6 and abs(gap - 0.24) < 1e-6


def test_synced_reader_aligns_frames_with_offsets(make_video):
    paths = {"cam0": make_video("c0.avi", 30), "cam1": make_video("c1.avi", 33)}
    seen = list(SyncedReader(paths, offsets={"cam1": 3}))
    assert len(seen) == 30
    for i, frames in seen:
        assert frame_number(frames["cam0"]) == i
        assert frame_number(frames["cam1"]) == i + 3


def test_synced_reader_step_and_masks(make_video):
    paths = {"cam0": make_video("c0.avi", 20, size=(160, 120))}
    seen = list(SyncedReader(paths, masks={"cam0": [[0, 0, 40, 30]]}, step=5))
    assert [i for i, _ in seen] == [0, 5, 10, 15]
    frame = seen[1][1]["cam0"]
    assert frame[:30, :40].max() == 0 and frame_number(frame[60:, 80:]) == 5


def info(size=(1920, 1080), fps=25.0, frames=100):
    return VideoInfo("x", size[0], size[1], "h264", fps, frames, frames / fps, fps, False, 1 / fps)


def test_group_and_resolution_checks():
    assert check_group({"cam0": info(), "cam1": info(), "cam2": info(frames=101)}) == []
    assert check_group({"cam0": info(), "cam1": info(fps=20.0), "cam2": info()})
    assert check_group({"cam0": info(), "cam1": info(frames=90), "cam2": info()})
    assert check_resolution(info(), {"session": info()}, "cam0") == []
    assert check_resolution(info(), {"session": info(size=(1280, 720))}, "cam0")


def test_contact_sheet_tiles_frames(make_video):
    sheet = contact_sheet(make_video("a.avi", 30, size=(320, 240)), n=6, cols=3, tile_width=200)
    assert sheet.shape == (2 * 150, 3 * 200, 3)
