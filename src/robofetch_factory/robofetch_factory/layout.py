"""Access to the generated factory data: named poses and maze path lengths.

Pure Python (no rclpy), so the fast training simulator and the ROS nodes read the SAME numbers.
The files are produced by scripts/generate_world.py from config/layout.yaml.
"""
import os
import sys

import yaml

_CONFIG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config")


def config_dir():
    """Source tree config/ when run from the workspace, else the installed share directory."""
    if os.path.isfile(os.path.join(_CONFIG_DIR, "poi.yaml")):
        return _CONFIG_DIR
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory("robofetch_factory"), "config")


def _load(name, directory=None):
    with open(os.path.join(directory or config_dir(), name)) as fh:
        return yaml.safe_load(fh)


def load_pois(directory=None):
    """{name: {"x", "y", "yaw", ...}} in the map frame (== Gazebo world frame)."""
    return _load("poi.yaml", directory)["poi"]


def load_path_matrix(directory=None):
    """{from: {to: metres}} shortest maze path between every pair of POIs."""
    return _load("path_matrix.yaml", directory)["length_m"]


def poi_cli():
    """`ros2 run robofetch_factory poi <name>` prints "x y yaw"; no name lists all POIs."""
    pois = load_pois()
    if len(sys.argv) < 2:
        for name, p in pois.items():
            print(f"{name} {p['x']} {p['y']} {p['yaw']}")
        return
    name = sys.argv[1]
    if name not in pois:
        sys.exit(f"unknown POI '{name}', known: {', '.join(pois)}")
    p = pois[name]
    print(f"{p['x']} {p['y']} {p['yaw']}")
