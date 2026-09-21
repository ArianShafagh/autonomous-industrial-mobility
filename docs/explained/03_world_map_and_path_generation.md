# 03 — World, map and path generation (`layout.yaml` → everything)

Files:
- `src/robofetch_factory/config/layout.yaml` is the **only hand-edited geometry**.
- `scripts/generate_world.py` is the generator (392 lines).
- Generated outputs (never edit by hand):
  - `src/robofetch_gazebo/worlds/factory_maze.sdf`: the Gazebo world
  - `src/robofetch_nav/maps/factory_maze.pgm` + `.yaml`: the Nav2 occupancy map
  - `src/robofetch_factory/config/poi.yaml`: named robot poses
  - `src/robofetch_factory/config/path_matrix.yaml`: maze driving distance between every POI pair
- `src/robofetch_factory/robofetch_factory/layout.py` has `load_pois()` and `load_path_matrix()`, with no ROS imports.

---

## 1. Why generate everything from one grid

The old project kept the Gazebo world and the navigation map as two hand-synchronised copies of the same rectangles. If they disagree by 10 cm, the robot localises wrongly or plans into walls. Here **both are derived from the same ASCII grid**, so:

- the map frame and the Gazebo world frame are identical by construction (POI coordinates are world coordinates),
- moving a wall moves it in the world, the map, the POIs and the distance matrix together,
- the AI's travel-cost model (path matrix) is computed on the same geometry the robot drives.

This is the **single source of truth** principle applied to geometry.

---

## 2. The layout file

```yaml
cell_size: 0.5          # metres per character
wall_height: 1.0
physics_step_s: 0.004   # Gazebo step (250 Hz) - see file 02 §7 for why not 1 ms
machine_height: 1.4
grid:
  - "#############################"     # row 0 = NORTH
  - "#MMM.....#.........#.....MMM#"
  - "#MMM.....#.........#.....MMM#"
  - "#AAA.....#...###...#.....BBB#"     # A pad (west), B pad (east)
  - "#........#...###...#........#"
  - "#........#...###...#........#"
  - "#....######........######...#"
  - "#....#..................#...#"
  - "#....#..................#...#"
  - "#....#....#######.......#...#"
  - "#.........#######...........#"
  - "#...........................#"
  - "#######..........###........#"
  - "#.....#..........#..........#"
  - "#.....#....####..#.....##MMM#"
  - "#..SS......####........##MMM#"     # S = charger
  - "#..SS....................CCC#"     # C pad
  - "#............DD.............#"     # D = delivery
  - "#.....#......DD..#..........#"
  - "#############################"     # row 19 = SOUTH
poi:
  A:        {zone: A, yaw_deg: 90}
  ...
  charger:  {zone: S, yaw_deg: 0}
```

Legend: `#` wall/rack, `M` machine (coloured by the nearest section), `A B C` pickup pads, `D` delivery pad, `S` charger pad, `.` free floor.

The grid is **29 columns × 20 rows × 0.5 m = 14.5 m × 10 m**.

### Design rules written into the file (lessons from the old project)

1. **Every passage is ≥ 3 cells = 1.5 m wide.** The robot's footprint radius is 0.22 m and the costmap inflation is 0.35 m. Anything under about 1.2 m can wedge it.
2. **Asymmetric layout.** AMCL localises by matching lidar scans to the map. In a symmetric room two places produce the same scan, and the particle filter can jump between them (*perceptual aliasing*).
3. **Alternative routes of different length** exist (central block, spurs, dead-end region), so path choice matters.

---

## 3. Coordinates: grid cell → world metres

`Layout.__init__` centres the hall on the world origin:

$$x_0 = -\frac{cols \cdot cell}{2} = -7.25\ \text{m}, \qquad y_{top} = \frac{rows \cdot cell}{2} = 5.0\ \text{m}$$

$$\text{cell\_center}(i, j) = \big(x_0 + (j + 0.5)\,cell,\ \ y_{top} - (i + 0.5)\,cell\big)$$

Row index `i` grows southward, so `y` decreases.

**Example:** the A pad cells are `(3,1), (3,2), (3,3)`, with centres x = −6.5, −6.0, −5.5 and y = 3.25. Their mean is **(−6.0, 3.25)**.

---

## 4. World generation (`write_world`)

### 4.1 Rectangle merging

Writing one Gazebo box per wall cell would give hundreds of models and slow physics. `merge_rectangles(mask)` greedily merges cells:

```
for each row i, scan columns j:
    if cell is wall:
        extend right while the next cell is wall        -> j..j1
        extend down while the whole row segment j..j1 is wall  -> i..i1
        emit rectangle (i, j, i1, j1), clear those cells
```

This is a greedy **rectangle cover** (not guaranteed minimal, but good). The result for this layout is **23 wall boxes** and **3 machine boxes**. With 7 floor pads that makes 31 models in total, plus the ground plane.

`rect_geometry` converts a cell rectangle to a centre and size in metres. Walls are 1.0 m tall. Machines are 1.4 m tall and coloured by `nearest_section`, which is the closest A/B/C cell by squared grid distance. Floor pads are 1 cm thick, **visual only** (`collide=False`), so the robot can drive onto them.

### 4.2 The SDF header

Physics step 0.004 s, real-time factor 1.0, and the standard gz-sim systems: Physics, SceneBroadcaster, UserCommands (needed so `obstacle.sh` can spawn/remove boxes at runtime), Contact, and Sensors (ogre2 render engine, needed for `gpu_lidar`).

---

## 5. Occupancy map generation (`FineGrid`, `write_map`)

### 5.1 Concept: occupancy grid map

A 2D **occupancy grid** splits space into cells with a probability of being occupied. Nav2's `map_server` loads a **PGM image + YAML**:

```yaml
image: factory_maze.pgm
mode: trinary               # only 3 states: free / occupied / unknown
resolution: 0.05            # metres per pixel
origin: [-7.750, -5.500, 0.0]   # world pose of the image's LOWER-LEFT pixel
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

The pixel value `p` becomes an occupancy probability `(255 − p)/255`:
- `0` (black) → 1.0 → **occupied**
- `254` (white) → ~0 → **free**
- `205` (grey) → 0.196 → neither above 0.65 nor below 0.196 → **unknown**

### 5.2 Rasterising the grid

```python
k = int(round(lay.cell / RES))                   # 0.5 / 0.05 = 10 pixels per cell
occ = [[c in "#M" for c in row] for row in grid] # 20 × 29 booleans
inner = np.kron(occ, np.ones((k, k)))            # 200 × 290 pixels
self.occ = np.pad(inner, m)                      # + 0.5 m margin = 10 px each side -> 220 × 310
self.known = np.pad(np.ones_like(inner), m, constant_values=False)
```

`np.kron` (Kronecker product) with a k×k block of ones **upsamples** each cell to a 10×10 block. The margin is *unknown* space around the hall.

- Map size: **310 × 220 px** = 15.5 m × 11 m.
- `origin_x = x0 − margin = −7.75`, `origin_y = −y_top − margin = −5.5`.

`to_px(x, y)` flips the row axis, because image row 0 is the *top* but map y grows *up*:

```python
col = int((x - origin_x) / RES)
row = h - 1 - int((y - origin_y) / RES)
```

### 5.3 Distance transform: clearance

```python
blocked = self.occ | ~self.known
self.clearance = ndimage.distance_transform_edt(~blocked) * RES
```

**Concept: Euclidean Distance Transform (EDT).** For every free pixel it computes the straight-line distance to the nearest blocked pixel. SciPy's `distance_transform_edt` does this exactly in linear time. Multiplying by 0.05 converts pixels to metres.

The resulting **clearance map** is used for three things:
1. POIs must have clearance ≥ 0.45 m (the robot can stand there).
2. Path lengths are computed only where clearance ≥ 0.40 m.
3. Pinch detection compares 0.40 m and 0.25 m spaces.

WP8's `compare_planners.py` reuses this class to measure how close each planner's path comes to walls.

---

## 6. POIs: where the robot stands (`compute_pois`)

For each POI:
1. Take the mean of its zone's cell centres. For A this is (−6.0, 3.25).
2. Look up the clearance there. For A it is **0.25 m**, because the pad is right against the machine, so the robot's body would touch it.
3. **Snap** to the nearest pixel with clearance ≥ `POI_CLEARANCE = 0.45 m`. This is a brute-force nearest search over all qualifying pixels (`argmin` of squared pixel distance).
4. Yaw comes from `yaw_deg` (90° = facing north, towards the machine for A/B/C).

Result (`poi.yaml`):

| POI | Zone centre | Clearance at centre | Snapped pose (x, y, yaw) | Snap distance | Final clearance |
|---|---|---|---|---|---|
| A | (−6.0, 3.25) | 0.25 m | (−5.975, 3.075, 90°) | 0.18 m | 0.45 m |
| B | (6.0, 3.25) | 0.25 m | (6.025, 3.075, 90°) | 0.18 m | 0.45 m |
| C | (6.0, −3.25) | 0.25 m | (6.025, −3.425, 90°) | 0.18 m | 0.45 m |
| delivery | (−0.25, −4.0) | 0.55 m | (−0.225, −3.975, 90°) | 0.04 m | 0.55 m |
| charger | (−5.25, −3.0) | 1.12 m | (−5.225, −2.975, 0°) | 0.04 m | 1.12 m |

The 0.04 m snaps for delivery/charger only come from rounding to pixel centres. `spawn: charger` tells `sim.launch.py` and `navigation.launch.py` (AMCL initial pose) where the robot starts.

---

## 7. The path-length matrix (`compute_matrix`)

The AI must know "how far is B from here?" **without running Nav2**, thousands of times per second. The generator pre-computes the drivable distance between all 5 POIs (a 5×5 matrix).

### 7.1 Step 1: passable space

`passable = clearance >= PATH_CLEARANCE (0.40 m)`. This is roughly the band where Nav2's inflated costmap keeps the robot centre, the inscribed radius 0.22 m plus a comfort margin. Using the raw free space would underestimate real driving distance by cutting corners the robot never cuts.

### 7.2 Step 2: 8-connected Dijkstra

```python
steps = [(-1,0,1), (1,0,1), (0,-1,1), (0,1,1),
         (-1,-1,√2), (-1,1,√2), (1,-1,√2), (1,1,√2)]
heap = [(0.0, start_row, start_col)]
while heap:
    d, r, c = heappop(heap)
    if d > dist[r, c]: continue                 # stale heap entry (lazy deletion)
    for dr, dc, cost in steps:
        if passable[nr, nc] and d + cost < dist[nr, nc]:
            dist[nr, nc] = d + cost
            parent[nr, nc] = r * w + c          # flat index of predecessor
            heappush(heap, (d + cost, nr, nc))
```

**Concept: Dijkstra's algorithm.** It finds shortest paths from one source in a graph with non-negative edge weights. A min-priority queue always expands the closest unexpanded node. Its complexity here is O(N log N) for N ≈ 68 200 pixels. One run per POI gives distances to *all* other POIs at once (single-source, all-targets).

**Why 8-connected?** With only 4 neighbours, paths are "Manhattan" staircases and a diagonal of length L costs L·√2. With 8 neighbours, diagonals cost √2 per step, which is still at most about 8 % longer than true Euclidean for arbitrary angles (the octile-distance error).

### 7.3 Step 3: string pulling (`smoothed_length`)

An 8-connected grid path zig-zags. A real planner/controller drives smooth, any-angle lines. **String pulling** removes unnecessary waypoints:

```python
anchors = [path[0]]; i = 0
while i < len(path) - 1:
    j = len(path) - 1
    while j > i + 1 and not line_of_sight(passable, path[i], path[j]):
        j -= 1                          # furthest point still visible from path[i]
    anchors.append(path[j]); i = j
length = sum of straight segments between anchors
```

`line_of_sight` samples `n = max(|Δrow|, |Δcol|) + 1` points along the segment and requires all to be passable. This is a discrete version of Bresenham's line check.

Picture it as a string laid along the grid path and pulled tight between the start and goal pegs. It wraps around corners and becomes straight everywhere else. The result approximates the **Euclidean shortest path** in the passable region, which is also what **Theta\*** computes directly during search (file 04, WP8).

### 7.4 Step 4: symmetrise

Greedy string pulling from A differs by a few cm from pulling from B, so the generator keeps `min(A→B, B→A)`. Physical travel is symmetric.

### 7.5 Result

```
            A       B       C   delivery  charger
A         0.00   18.28   15.56    10.01     9.04
B        18.28    0.00    9.24    10.26    15.18
C        15.56    9.24    0.00     6.30    11.27
delivery 10.01   10.26    6.30     0.00     5.10
charger   9.04   15.18   11.27     5.10     0.00
```

**Verification (HANDOVER WP1):** Nav2's planner lengths vs this matrix differ by at most **2.0 %** over all 10 pairs. A driven 7-leg tour covered **57.27 m** of ground truth against **57.27 m** from the matrix.

Things to notice for the AI:
- A↔B is 18.28 m, the longest trip, so collecting A and B in one trip is expensive.
- C is close to delivery (6.3 m) but slow (5 units/h) and heavy (1.5 kg/unit).
- The charger is 5.1 m from delivery. The first draft had it 1.4 m away, which would make charging trips nearly free and remove a real trade-off, so it was moved (WP1 decision).

---

## 8. Pinch detection: the most important safety check of the generator

### 8.1 The failure it prevents (a real bug, WP1)

The first maze had a **0.5 m slot** between two wall segments. Nav2's planner treats only cells within the **inscribed radius (0.22 m)** as lethal. A 0.5 m gap leaves a 0.06 m-wide lethal-free strip, so the **planner routed through it**. The controller, which must respect the whole footprint plus inflation, could not pass. The robot got stuck and re-routed the long way:
- Nav2 planned B→C = 10.47 m while the matrix said 22.89 m (−54 %).
- The robot drove **24.4 m instead of about 10 m** to reach B.

### 8.2 How detection works

Compare shortest paths in two spaces that differ **only in narrow slots**:

```python
loose_mask = clearance >= PINCH_CLEARANCE                      # 0.25 m: slots included
grow = round((PATH_CLEARANCE - PINCH_CLEARANCE) / RES) + 1      # (0.40-0.25)/0.05 + 1 = 4 px
wide_mask = binary_dilation(clearance >= PATH_CLEARANCE, iterations=grow) & loose_mask
```

- `loose` is every point with ≥ 0.25 m clearance. A gap between 0.5 m and 0.8 m wide is passable here.
- `wide` takes the comfortable space (≥ 0.40 m) and **grows it by 4 px (0.20 m)** with morphological dilation, then intersects with `loose`. Growing lets paths hug corners down to 0.25 m clearance just like `loose`. But a slot narrower than 0.8 m has *no* ≥ 0.40 m pixels inside it, so dilation can only reach 0.20 m into it from each end and never connects through a long slot.

So both spaces allow the same corner-hugging, and they differ only where a narrow passage exists. If a route is **> 5 % shorter in `loose` than in `wide`**, it must go through a pinch, and generation aborts:

```
PINCH: B<->C is 22.89 m for the robot but 10.47 m through a gap narrower than 0.8 m - widen it or close it in layout.yaml
```

**Concept: morphological dilation.** Every True pixel spreads to its neighbours, once per iteration. It is used here to model how far a path can "reach" from comfortable space.

**Why not just compare raw lengths at two clearances?** The first attempt did that and falsely flagged A↔B (−5.3 %) and A↔charger (−6.6 %). Those differences were only tighter corner cutting, not slots. The dilation trick separates "cuts corners closer" from "goes through a gap". This was verified by feeding the old broken layout back in: it flags exactly B↔C, B↔delivery and B↔charger, the three pairs Nav2 got wrong.

---

## 9. `layout.py`: runtime access

```python
def config_dir():
    if os.path.isfile(os.path.join(_CONFIG_DIR, "poi.yaml")):   # source tree
        return _CONFIG_DIR
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory("robofetch_factory"), "config")
```

This works both from `src/` (fast simulator, tools) and from an installed package (ROS nodes). `ros2 run robofetch_factory poi B` prints `6.025 3.075 1.5708`, which `goto.sh`, `set_pose.sh` and `obstacle.sh` use.

---

## 10. Worked example: from the grid to a decision

1. The robot is at `delivery` with 4 units of B on board. The AI considers `PICKUP:C`.
2. `matrix["delivery"]["C"] = 6.30 m`, which came from Dijkstra + string pulling on the clearance-0.40 space.
3. `robot_model.trip_energy_wh(p, 6.30, payload)` gives the energy. Time is `6.30 / 0.39 = 16.2 s` of driving.
4. The symbolic safety rule forward-simulates `delivery → C → charger` = 6.30 + 11.27 m.
5. If chosen, the executor sends Nav2 to `poi["C"] = (6.025, −3.425, 90°)`, the snapped pose with 0.45 m clearance.
6. Nav2 plans on `factory_maze.pgm` (same geometry), drives about 6.3 m, and AMCL confirms arrival within 0.35 m.

Every number in that chain comes from `layout.yaml`.

---

## 11. Key concepts of this section

- Occupancy grid maps and the PGM/YAML format (trinary mode, thresholds, origin)
- Kronecker upsampling and image vs map axis flip
- Euclidean distance transform → clearance map
- Configuration space: planning for a point robot in obstacle space grown by the robot radius (clearance ≥ r is exactly that)
- Dijkstra single-source shortest paths, 8-connectivity, lazy deletion in the heap
- Line-of-sight string pulling ≈ any-angle shortest path
- Morphological dilation to separate "narrow passage" from "corner cutting"
- Greedy rectangle merging to reduce simulator load
- Perceptual aliasing and why asymmetry helps localisation

---

## 12. Improvements and technologies for this section

| Idea | Why | How |
|---|---|---|
| **Path matrix per payload/through-traffic** | The matrix is pure distance. A real AMR drives slower through narrow or cluttered areas, so *time* ≠ distance/constant speed | Weight Dijkstra edges by a speed map (lower in narrow corridors: speed ∝ clearance) and store a *time matrix* next to the length matrix |
| **Dynamic path costs** | If a corridor is blocked (obstacle.sh), the matrix is wrong until the robot fails a drive. The AI cannot anticipate this | Query Nav2 `ComputePathToPose` for the candidate destinations at decision time (≈ 8–20 ms per plan per WP8), or update the matrix from the global costmap every N seconds |
| **More POIs / a graph instead of 5 points** | Real factories have many stations. The current AI vocabulary is hard-wired to A/B/C | Generate POIs from all letter zones automatically; represent the factory as a graph (nodes = POIs, edges = matrix). This also enables a GNN policy (file 16) |
| **Visibility graph or exact Euclidean shortest path** | String pulling is greedy from one end and only approximately optimal | Build a visibility graph on the corners of the dilated obstacles and run Dijkstra on it (exact for polygonal obstacles), or use the **Theta\*** algorithm directly on the grid |
| **Validate the SDF/map automatically in tests** | The generator is only checked when someone runs it | pytest that regenerates into a temp dir and asserts: map size, every POI clearance ≥ 0.45, matrix symmetric, triangle inequality holds, no pinch |
| **Procedural layout generation** | One maze means the AI may overfit to one geometry | Randomise walls under the same design rules (passages ≥ 1.5 m, no pinches) and train/evaluate on many layouts (domain randomisation) |
| **Use real CAD / IFC building data** | Industrial plants have floor plans | Rasterise a DXF/IFC floor plan into the same grid format; the rest of the pipeline stays unchanged |
