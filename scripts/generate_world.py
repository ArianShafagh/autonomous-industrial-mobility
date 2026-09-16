#!/usr/bin/env python3
"""Generate the factory world, the Nav2 map, the POI poses and the path-length matrix
from ONE file: src/robofetch_factory/config/layout.yaml.

The old project kept the world (SDF) and the map (generate_map.py) as two hand-synchronised
copies of the same rectangles. Here both are derived from the same ASCII grid, so the map frame
and the Gazebo world frame are identical by construction and waypoints are world coordinates.

Outputs
  src/robofetch_gazebo/worlds/factory_maze.sdf
  src/robofetch_nav/maps/factory_maze.pgm + .yaml
  src/robofetch_factory/config/poi.yaml          named poses (x, y, yaw) the robot drives to
  src/robofetch_factory/config/path_matrix.yaml  maze path length (m) between every POI pair

Run:  venv/bin/python scripts/generate_world.py
"""
import heapq
import math
import os

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage

WS = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
LAYOUT = os.path.join(WS, "src", "robofetch_factory", "config", "layout.yaml")
OUT_WORLD = os.path.join(WS, "src", "robofetch_gazebo", "worlds", "factory_maze.sdf")
OUT_MAP_DIR = os.path.join(WS, "src", "robofetch_nav", "maps")
OUT_POI = os.path.join(WS, "src", "robofetch_factory", "config", "poi.yaml")
OUT_MATRIX = os.path.join(WS, "src", "robofetch_factory", "config", "path_matrix.yaml")

RES = 0.05            # map resolution, m/px (matches the Nav2 costmaps)
MAP_MARGIN = 0.5      # unknown border around the hall in the map image
# Robot footprint radius is 0.22 m. A pose must keep this much clearance from any obstacle so the
# robot can stand there without its inflated footprint touching the wall or the machine.
POI_CLEARANCE = 0.45
# Clearance used for the path-length estimate: inscribed radius + a margin, roughly where the
# Nav2 planner's cost gradient keeps the robot. Too small underestimates real paths.
PATH_CLEARANCE = 0.40
# Nav2 only treats cells within the inscribed radius (0.22 m) as lethal, so its planner will
# happily route through a gap the robot cannot actually drive. Any route that becomes shorter at
# this looser clearance passes through such a PINCH and is rejected - that is how the first
# layout sent the robot into a 0.5 m slot, fail, and re-route 13 m the long way.
PINCH_CLEARANCE = 0.25
PINCH_TOLERANCE = 0.05       # 5 % shorter at the loose clearance counts as a pinch

OBSTACLE = "#M"


class Layout:
    def __init__(self, path):
        with open(path) as fh:
            cfg = yaml.safe_load(fh)
        self.cfg = cfg
        self.grid = cfg["grid"]
        widths = {len(r) for r in self.grid}
        if len(widths) != 1:
            raise SystemExit(f"layout grid rows have different widths: {sorted(widths)}")
        self.rows = len(self.grid)
        self.cols = widths.pop()
        self.cell = float(cfg["cell_size"])
        # Centre the hall on the world origin.
        self.x0 = -self.cols * self.cell / 2.0
        self.y_top = self.rows * self.cell / 2.0

    def cell_center(self, i, j):
        return (self.x0 + (j + 0.5) * self.cell, self.y_top - (i + 0.5) * self.cell)

    def cells(self, ch):
        return [(i, j) for i, row in enumerate(self.grid) for j, c in enumerate(row) if c == ch]


def merge_rectangles(mask):
    """Greedy merge of True cells into axis-aligned rectangles (i0, j0, i1, j1), inclusive."""
    mask = mask.copy()
    rects = []
    rows, cols = mask.shape
    for i in range(rows):
        j = 0
        while j < cols:
            if not mask[i, j]:
                j += 1
                continue
            j1 = j
            while j1 + 1 < cols and mask[i, j1 + 1]:
                j1 += 1
            i1 = i
            while i1 + 1 < rows and mask[i1 + 1, j:j1 + 1].all():
                i1 += 1
            mask[i:i1 + 1, j:j1 + 1] = False
            rects.append((i, j, i1, j1))
            j = j1 + 1
    return rects


# ------------------------------------------------------------------------------------ world
def box_model(name, x, y, z, sx, sy, sz, rgb, collide=True):
    color = f"{rgb[0]} {rgb[1]} {rgb[2]} 1"
    collision = (f"<collision name='c'><geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size>"
                 f"</box></geometry></collision>") if collide else ""
    return (f"    <model name='{name}'><static>true</static>"
            f"<pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose><link name='l'>{collision}"
            f"<visual name='v'><geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box>"
            f"</geometry><material><ambient>{color}</ambient><diffuse>{color}</diffuse>"
            f"</material></visual></link></model>\n")


def rect_geometry(lay, rect):
    i0, j0, i1, j1 = rect
    xa, ya = lay.cell_center(i0, j0)
    xb, yb = lay.cell_center(i1, j1)
    sx = (j1 - j0 + 1) * lay.cell
    sy = (i1 - i0 + 1) * lay.cell
    return (xa + xb) / 2.0, (ya + yb) / 2.0, sx, sy


def nearest_section(lay, i, j):
    best, best_d = None, float("inf")
    for ch in "ABC":
        for (a, b) in lay.cells(ch):
            d = (a - i) ** 2 + (b - j) ** 2
            if d < best_d:
                best, best_d = ch, d
    return best


def write_world(lay):
    colors = lay.cfg["colors"]
    arr = np.array([list(r) for r in lay.grid])
    parts = []

    for n, rect in enumerate(merge_rectangles(arr == "#")):
        x, y, sx, sy = rect_geometry(lay, rect)
        h = lay.cfg["wall_height"]
        parts.append(box_model(f"wall_{n}", x, y, h / 2, sx, sy, h, colors["wall"]))

    for n, rect in enumerate(merge_rectangles(arr == "M")):
        x, y, sx, sy = rect_geometry(lay, rect)
        h = lay.cfg["machine_height"]
        section = nearest_section(lay, rect[0], rect[1])
        parts.append(box_model(f"machine_{section}_{n}", x, y, h / 2, sx, sy, h,
                               colors[section]))

    # Floor pads: visual only, never obstacles.
    for ch in "ABCDS":
        for n, rect in enumerate(merge_rectangles(arr == ch)):
            x, y, sx, sy = rect_geometry(lay, rect)
            parts.append(box_model(f"pad_{ch}_{n}", x, y, 0.005, sx, sy, 0.01, colors[ch],
                                   collide=False))

    header = f"""<?xml version="1.0" ?>
<!-- GENERATED by scripts/generate_world.py from robofetch_factory/config/layout.yaml.
     Do not edit by hand. Hall {lay.cols * lay.cell:.1f} x {lay.rows * lay.cell:.1f} m, maze of walls,
     three production sections (A, B, C), one delivery point (green) and one charger (blue). -->
<sdf version="1.10">
  <world name="factory_maze">
    <physics name="step" type="ignored">
      <max_step_size>{lay.cfg["physics_step_s"]}</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.7 0.8 0.9 1</background>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.3 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
          <material><ambient>0.85 0.85 0.85 1</ambient><diffuse>0.85 0.85 0.85 1</diffuse></material>
        </visual>
      </link>
    </model>
"""
    with open(OUT_WORLD, "w") as fh:
        fh.write(header + "".join(parts) + "  </world>\n</sdf>\n")
    n_walls = sum(p.count("name='wall_") for p in parts)
    return n_walls, len(parts)


# -------------------------------------------------------------------------------------- map
class FineGrid:
    """The layout rasterised at RES, with the map margin, in map_server orientation."""

    def __init__(self, lay):
        self.lay = lay
        k = int(round(lay.cell / RES))
        occ = np.array([[c in OBSTACLE for c in r] for r in lay.grid])
        inner = np.kron(occ, np.ones((k, k), dtype=bool))
        m = int(round(MAP_MARGIN / RES))
        self.m = m
        self.occ = np.pad(inner, m, constant_values=False)
        self.known = np.pad(np.ones_like(inner), m, constant_values=False)
        self.h, self.w = self.occ.shape
        self.origin_x = lay.x0 - MAP_MARGIN
        self.origin_y = -lay.y_top - MAP_MARGIN
        # Clearance to the nearest obstacle OR unknown border, in metres.
        blocked = self.occ | ~self.known
        self.clearance = ndimage.distance_transform_edt(~blocked) * RES

    def to_px(self, x, y):
        col = int((x - self.origin_x) / RES)
        row = self.h - 1 - int((y - self.origin_y) / RES)
        return row, col

    def to_world(self, row, col):
        return (float(self.origin_x + (col + 0.5) * RES),
                float(self.origin_y + (self.h - 1 - row + 0.5) * RES))


def write_map(fg):
    img = np.full(fg.occ.shape, 205, dtype=np.uint8)   # unknown
    img[fg.known] = 254                                  # free
    img[fg.occ] = 0                                      # occupied
    Image.fromarray(img).save(os.path.join(OUT_MAP_DIR, "factory_maze.pgm"))
    with open(os.path.join(OUT_MAP_DIR, "factory_maze.yaml"), "w") as fh:
        fh.write("# GENERATED by scripts/generate_world.py - do not edit by hand.\n"
                 "image: factory_maze.pgm\n"
                 "mode: trinary\n"
                 f"resolution: {RES}\n"
                 f"origin: [{fg.origin_x:.3f}, {fg.origin_y:.3f}, 0.0]\n"
                 "negate: 0\n"
                 "occupied_thresh: 0.65\n"
                 "free_thresh: 0.196\n")


# -------------------------------------------------------------------------------------- POIs
def compute_pois(lay, fg):
    pois = {}
    free = fg.clearance >= POI_CLEARANCE
    fr, fc = np.where(free)
    for name, spec in lay.cfg["poi"].items():
        cells = lay.cells(spec["zone"])
        if not cells:
            raise SystemExit(f"POI {name}: zone '{spec['zone']}' not found in the grid")
        cx = float(np.mean([lay.cell_center(i, j)[0] for i, j in cells]))
        cy = float(np.mean([lay.cell_center(i, j)[1] for i, j in cells]))
        r, c = fg.to_px(cx, cy)
        # Snap to the nearest pixel where the robot fits (moves it off the machine face).
        idx = int(np.argmin((fr - r) ** 2 + (fc - c) ** 2))
        x, y = fg.to_world(fr[idx], fc[idx])
        pois[name] = {"x": round(x, 3), "y": round(y, 3),
                      "yaw": round(math.radians(spec["yaw_deg"]), 4),
                      "zone": spec["zone"],
                      "clearance_m": round(float(fg.clearance[fr[idx], fc[idx]]), 2),
                      "snap_m": round(math.hypot(x - cx, y - cy), 2)}
    return pois


# ------------------------------------------------------------------------------ path lengths
def dijkstra(passable, start):
    """8-connected shortest distance (pixels) from start over passable pixels."""
    h, w = passable.shape
    dist = np.full((h, w), np.inf)
    parent = -np.ones((h, w), dtype=np.int64)
    dist[start] = 0.0
    heap = [(0.0, start[0], start[1])]
    steps = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
             (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
             (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))]
    while heap:
        d, r, c = heapq.heappop(heap)
        if d > dist[r, c]:
            continue
        for dr, dc, cost in steps:
            nr, nc = r + dr, c + dc
            if 0 <= nr < h and 0 <= nc < w and passable[nr, nc]:
                nd = d + cost
                if nd < dist[nr, nc]:
                    dist[nr, nc] = nd
                    parent[nr, nc] = r * w + c
                    heapq.heappush(heap, (nd, nr, nc))
    return dist, parent


def line_of_sight(passable, a, b):
    n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1]))) + 1
    rows = np.linspace(a[0], b[0], n).round().astype(int)
    cols = np.linspace(a[1], b[1], n).round().astype(int)
    return bool(passable[rows, cols].all())


def smoothed_length(passable, path):
    """String-pull an 8-connected path: the length a smooth any-angle planner would drive."""
    anchors = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not line_of_sight(passable, path[i], path[j]):
            j -= 1
        anchors.append(path[j])
        i = j
    length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(anchors, anchors[1:]))
    return length * RES


def compute_matrix(fg, pois, passable=None):
    if passable is None:
        passable = fg.clearance >= PATH_CLEARANCE
    names = list(pois)
    px = {n: fg.to_px(pois[n]["x"], pois[n]["y"]) for n in names}
    matrix = {n: {} for n in names}
    for a in names:
        if not passable[px[a]]:
            raise SystemExit(f"POI {a} is not on passable ground at clearance {PATH_CLEARANCE}")
        dist, parent = dijkstra(passable, px[a])
        for b in names:
            if a == b:
                matrix[a][b] = 0.0
                continue
            if not np.isfinite(dist[px[b]]):
                raise SystemExit(f"no path from {a} to {b} - the maze is disconnected")
            path, node = [], px[b][0] * fg.w + px[b][1]
            while node >= 0:
                path.append((node // fg.w, node % fg.w))
                node = parent[path[-1]]
            path.reverse()
            matrix[a][b] = smoothed_length(passable, path)
    # String pulling is greedy from the start point, so A->B and B->A can differ by a few cm.
    # Travel cost is physically symmetric: keep the shorter of the two.
    for a in names:
        for b in names:
            matrix[a][b] = min(matrix[a][b], matrix[b][a])
    return {a: {b: round(v, 2) for b, v in row.items()} for a, row in matrix.items()}


def main():
    lay = Layout(LAYOUT)
    os.makedirs(OUT_MAP_DIR, exist_ok=True)
    walls, models = write_world(lay)
    fg = FineGrid(lay)
    write_map(fg)
    pois = compute_pois(lay, fg)
    matrix = compute_matrix(fg, pois)
    # Compare two spaces that differ ONLY in narrow slots: both may hug corners down to
    # PINCH_CLEARANCE, but `wide` keeps only pixels within reach of the comfortable space, so a
    # slot narrower than 2*PATH_CLEARANCE exists in `loose` and not in `wide`.
    loose_mask = fg.clearance >= PINCH_CLEARANCE
    grow = int(round((PATH_CLEARANCE - PINCH_CLEARANCE) / RES)) + 1
    wide_mask = ndimage.binary_dilation(fg.clearance >= PATH_CLEARANCE, iterations=grow) & loose_mask
    wide = compute_matrix(fg, pois, wide_mask)
    loose = compute_matrix(fg, pois, loose_mask)
    pinches = [(a, b, wide[a][b], loose[a][b]) for a in matrix for b in matrix
               if a < b and loose[a][b] < wide[a][b] * (1 - PINCH_TOLERANCE)]
    if pinches:
        for a, b, ok, short in pinches:
            print(f"PINCH: {a}<->{b} is {ok:.2f} m for the robot but {short:.2f} m through a gap "
                  f"narrower than {2 * PATH_CLEARANCE:.1f} m - widen it or close it in layout.yaml")
        raise SystemExit(1)

    note = "# GENERATED by scripts/generate_world.py from layout.yaml - do not edit by hand.\n"
    with open(OUT_POI, "w") as fh:
        fh.write(note + "# x, y in metres (map == world frame), yaw in radians.\n")
        yaml.safe_dump({"frame_id": "map", "spawn": "charger", "poi": pois}, fh, sort_keys=False)
    with open(OUT_MATRIX, "w") as fh:
        fh.write(note + f"# Shortest maze path (m), string-pulled, clearance >= {PATH_CLEARANCE} m.\n")
        yaml.safe_dump({"clearance_m": PATH_CLEARANCE, "length_m": matrix}, fh, sort_keys=False)

    print(f"hall {lay.cols * lay.cell:.1f} x {lay.rows * lay.cell:.1f} m, "
          f"{walls} wall boxes, {models} models -> {os.path.relpath(OUT_WORLD, WS)}")
    print(f"map {fg.w}x{fg.h} px, origin ({fg.origin_x:.2f}, {fg.origin_y:.2f})")
    for n, p in pois.items():
        print(f"  {n:9s} x={p['x']:6.2f} y={p['y']:6.2f} yaw={math.degrees(p['yaw']):5.0f}  "
              f"clearance {p['clearance_m']} m  snapped {p['snap_m']} m")
    names = list(pois)
    print("path length matrix (m):")
    print("           " + "".join(f"{n:>10s}" for n in names))
    for a in names:
        print(f"  {a:9s}" + "".join(f"{matrix[a][b]:10.2f}" for b in names))


if __name__ == "__main__":
    main()
