import os
import sys

import cv2
import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def write_video(path, n_frames, size=(160, 120), fps=25.0):
    """A small MJPG video; frame i is flat grey 7*i (frame_number reads it back up to frame 36)."""
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"), fps, size)
    assert writer.isOpened()
    for i in range(n_frames):
        writer.write(np.full((size[1], size[0], 3), (7 * i) % 256, np.uint8))
    writer.release()
    return str(path)


def frame_number(frame):
    return int(round(float(frame.mean()) / 7))


@pytest.fixture
def make_video(tmp_path):
    def make(name, n_frames, **kw):
        return write_video(tmp_path / name, n_frames, **kw)
    return make
