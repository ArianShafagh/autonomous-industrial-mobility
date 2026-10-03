"""Camera: projection and undistortion must be exact inverses for every lens model."""
import numpy as np
import pytest

from mocap.camera import Camera

K = [[600.0, 0, 960], [0, 600.0, 540], [0, 0, 1]]
DIST = {
    "pinhole": [-0.2, 0.05, 0.001, -0.001, 0.0],
    "rational": [-0.2, 0.05, 0.001, -0.001, 0.0, 0.01, 0.002, 0.0],
    "fisheye": [0.05, -0.01, 0.002, -0.0005],
}


def look_at(eye, target, up=(0, 0, 1)):
    """world->camera R, t for a camera at `eye` looking at `target` (camera y points down)."""
    eye, target = np.asarray(eye, float), np.asarray(target, float)
    z = target - eye
    z /= np.linalg.norm(z)
    x = np.cross(z, up)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    R = np.vstack([x, y, z])
    return R, -R @ eye


def ceiling_camera(model):
    """A camera 2.6 m up in a room corner looking at the middle of the floor, like the real rig."""
    R, t = look_at([0.0, 0.0, 2.6], [2.0, 2.0, 0.8])
    return Camera("cam0", model, K, DIST[model], (1920, 1080), R, t)


@pytest.mark.parametrize("model", ["pinhole", "rational", "fisheye"])
def test_project_then_undistort_gives_the_ray(model):
    cam = ceiling_camera(model)
    rng = np.random.default_rng(0)
    X = rng.uniform([1.0, 1.0, 0.0], [3.0, 3.0, 1.8], size=(50, 3))
    uv = cam.project(X)
    Xc = (cam.R @ X.T).T + cam.t
    expected = Xc[:, :2] / Xc[:, 2:]
    assert np.allclose(cam.undistort(uv), expected, atol=1e-6)


def test_camera_centre_is_where_it_was_placed():
    assert np.allclose(ceiling_camera("fisheye").center, [0.0, 0.0, 2.6])


def test_nan_points_stay_nan():
    cam = ceiling_camera("fisheye")
    X = np.array([[2.0, 2.0, 1.0], [np.nan, 0, 0]])
    uv = cam.project(X)
    assert np.isfinite(uv[0]).all() and np.isnan(uv[1]).all()
    assert np.isnan(cam.undistort(uv)[1]).all()


def test_yaml_round_trip(tmp_path):
    cam = ceiling_camera("fisheye")
    cam.rms = 0.31
    cam.save(tmp_path / "c.yaml")
    back = Camera.load(tmp_path / "c.yaml")
    assert back.model == "fisheye" and back.size == (1920, 1080) and back.rms == 0.31
    assert np.allclose(back.project([[2, 2, 1]]), cam.project([[2, 2, 1]]))


def test_rejects_bad_models():
    with pytest.raises(ValueError):
        Camera("c", "kb8", K, DIST["fisheye"], (10, 10))
    with pytest.raises(ValueError):
        Camera("c", "fisheye", K, DIST["pinhole"], (10, 10))
