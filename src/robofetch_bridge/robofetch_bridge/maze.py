"""The maze, drawn for the dashboard, with the robot on it.

The picture is built from the same `layout.yaml` that produced the Gazebo world and the Nav2 map,
so it can never drift out of step with what the robot drives in. Walls and machines are merged
into runs of cells (a few dozen rectangles instead of ~580), the section pads keep their layout
colours, and the robot's live pose is drawn on top with a short trail.

    from robofetch_bridge.maze import MazeView
    MazeView().svg(robot_xy=(1.2, -3.4), trail=[...])   -> <svg ...>...</svg>
"""
import html
import os

import yaml

POI_LABELS = {"A": "A", "B": "B", "C": "C", "D": "delivery", "S": "charger"}
ZONE_CHARS = ("A", "B", "C", "D", "S")


def _config_dir():
    here = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    local = os.path.join(here, "robofetch_factory", "config")
    if os.path.isfile(os.path.join(local, "layout.yaml")):
        return local
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory("robofetch_factory"), "config")


def _rgb(triple):
    return "#%02x%02x%02x" % tuple(max(0, min(255, round(c * 255))) for c in triple)


def _merge_rows(cells, cols):
    """Neighbouring cells of one row merged into (i, j0, j1) runs - far fewer rectangles."""
    runs = []
    for i, j in sorted(cells):
        if runs and runs[-1][0] == i and runs[-1][2] == j - 1:
            runs[-1][2] = j
        else:
            runs.append([i, j, j])
    return runs


class MazeView:
    """Reads layout.yaml once and renders the maze as SVG in metres (world units)."""

    def __init__(self, config_dir=None):
        directory = config_dir or _config_dir()
        with open(os.path.join(directory, "layout.yaml")) as fh:
            cfg = yaml.safe_load(fh)
        self.grid = cfg["grid"]
        self.cell = float(cfg["cell_size"])
        self.colors = cfg.get("colors", {})
        self.rows, self.cols = len(self.grid), len(self.grid[0])
        # Same framing as generate_world.py: the hall is centred on the world origin.
        self.x0 = -self.cols * self.cell / 2.0
        self.y_top = self.rows * self.cell / 2.0
        self.width = self.cols * self.cell
        self.height = self.rows * self.cell
        self._static = self._draw_static()

    # ------------------------------------------------------------------- world -> picture
    def point(self, x, y):
        """World metres -> picture coordinates (y grows downwards in SVG)."""
        return (x - self.x0, self.y_top - y)

    def _cells(self, *chars):
        return [(i, j) for i, row in enumerate(self.grid)
                for j, c in enumerate(row) if c in chars]

    def _rects(self, cells, fill, opacity=1.0):
        out = []
        for i, j0, j1 in _merge_rows(cells, self.cols):
            x = j0 * self.cell
            y = i * self.cell
            out.append(f'<rect x="{x:.2f}" y="{y:.2f}" width="{(j1 - j0 + 1) * self.cell:.2f}" '
                       f'height="{self.cell:.2f}" fill="{fill}" opacity="{opacity:g}"/>')
        return out

    def _draw_static(self):
        parts = [f'<rect x="0" y="0" width="{self.width:.2f}" height="{self.height:.2f}" '
                 f'fill="var(--maze-floor, #f7f7f5)"/>']
        for char in ZONE_CHARS:                       # coloured pads under the machines
            cells = self._cells(char)
            if cells:
                parts += self._rects(cells, _rgb(self.colors.get(char, [0.6, 0.6, 0.6])), 0.35)
        parts += self._rects(self._cells("#"), _rgb(self.colors.get("wall", [0.5, 0.5, 0.55])))
        parts += self._rects(self._cells("M"), "var(--maze-machine, #6b6b73)")
        for char, label in POI_LABELS.items():        # one label per zone, at its centre
            cells = self._cells(char)
            if not cells:
                continue
            cx = (sum(j for _, j in cells) / len(cells) + 0.5) * self.cell
            cy = (sum(i for i, _ in cells) / len(cells) + 0.5) * self.cell
            parts.append(f'<text x="{cx:.2f}" y="{cy:.2f}" font-size="0.75" text-anchor="middle" '
                         f'dominant-baseline="central" fill="var(--maze-label, #26262a)">'
                         f'{html.escape(label)}</text>')
        return "".join(parts)

    # ------------------------------------------------------------------------------ render
    def svg(self, robot_xy=None, trail=(), heading=None):
        """The maze with the robot's position (and where it has been) drawn on top."""
        parts = [self._static]
        points = [self.point(x, y) for x, y in trail if x is not None and y is not None]
        if len(points) > 1:
            path = " ".join(f"{x:.2f},{y:.2f}" for x, y in points)
            parts.append(f'<polyline points="{path}" fill="none" stroke="var(--maze-trail, '
                         f'#2a78d6)" stroke-width="0.09" stroke-opacity="0.45" '
                         f'stroke-linejoin="round" stroke-linecap="round"/>')
        if robot_xy and robot_xy[0] is not None:
            cx, cy = self.point(robot_xy[0], robot_xy[1])
            parts.append(f'<circle cx="{cx:.2f}" cy="{cy:.2f}" r="0.30" '
                         f'fill="var(--maze-robot, #2a78d6)" stroke="white" stroke-width="0.08"/>')
        else:
            parts.append(f'<text x="{self.width / 2:.2f}" y="{self.height / 2:.2f}" '
                         f'font-size="0.9" text-anchor="middle" fill="var(--maze-label, #26262a)" '
                         f'opacity="0.55">waiting for the robot\'s position</text>')
        return (f'<svg viewBox="0 0 {self.width:.2f} {self.height:.2f}" '
                f'class="maze" role="img" aria-label="factory maze with the robot\'s position" '
                f'preserveAspectRatio="xMidYMid meet">{"".join(parts)}</svg>')
