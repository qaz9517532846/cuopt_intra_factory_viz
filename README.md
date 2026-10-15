# cuopt_intra_factory_viz

Turns NVIDIA cuOpt's official [Intra-factory Transport](https://docs.nvidia.com/cuopt/user-guide/latest/cuopt-python/routing/routing-examples.html) example into a package that actually calls cuOpt to solve, then animates the result with ROS2 waypoints + tf2: on startup it solves a 6-pair pickup-delivery VRP (2 AMRs, capacity 2 each) over a 10-waypoint factory floor graph, expands the solved route into a full waypoint sequence, and drives the vehicles around in RViz2 using a differential-drive kinematic model (in-place rotation or straight-line translation only — never both at once). Vehicles actually stop to load/unload at each stop, and the package tracks which shipments each vehicle is currently carrying.

## System Requirements

- **GPU**: NVIDIA GPU + CUDA driver (this package's build requires `LANGUAGES CXX CUDA`)
- **CMake**: >= 3.30.4 (needed by RMM/RAFT's CMake config; the apt-packaged system CMake is usually too old — see [Installation](#installation))
- Same `libcuopt` / RAFT / RMM conda env as `cuopt_routing_node` in this workspace — if that package builds, this one will too

## Built With

- [ROS Jazzy](https://docs.ros.org/en/jazzy/) under Ubuntu 24.04 LTS
- [NVIDIA cuOpt](https://docs.nvidia.com/cuopt/user-guide/latest/introduction.html) (`libcuopt`, the `cuopt::routing` C++ template API, and `cuopt::distance_engine::waypoint_matrix_t`)
- [RAPIDS RAFT](https://github.com/rapidsai/raft) / [RMM](https://github.com/rapidsai/rmm)
- [`rviz_default_plugins`](https://docs.ros.org/en/jazzy/p/rviz_default_plugins/) (MarkerArray / TF display)

## Architecture

| File | Content | Needs CUDA/libcuopt |
|---|---|---|
| [`waypoint_graph.hpp/.cpp`](include/cuopt_intra_factory_viz/waypoint_graph.hpp) | The 10 waypoints' coordinates/roles (depot / station / transit junction) and edge connectivity. Edge length is always the Euclidean distance between the two endpoints, computed dynamically — never a hardcoded number. | No |
| [`shortest_path.hpp/.cpp`](include/cuopt_intra_factory_viz/shortest_path.hpp) | A small Dijkstra shortest-path utility (not currently used by the demo nodes; kept for scenarios that need a manually-routed path) | No |
| [`diff_drive_path.hpp/.cpp`](include/cuopt_intra_factory_viz/diff_drive_path.hpp) | The differential-drive motion simulator: splits a sequence of waypoint ids into "rotate" / "translate" / "dwell" primitives, executed with acceleration-limited trapezoidal/triangular velocity profiles | No |
| [`cuopt_solver.hpp`](include/cuopt_intra_factory_viz/cuopt_solver.hpp) / [`cuopt_solver.cu`](src/cuopt_solver.cu) | The **only** place in this package that links CUDA/RAFT/RMM/`libcuopt`: build the graph → compress it into a cost matrix → solve the 6-pair pickup-delivery VRP → expand the result back into a full path → pin down exactly where each dwell event happens | **Yes** |
| [`waypoint_marker_publisher_node.cpp`](src/waypoint_marker_publisher_node.cpp) | Pure visualization node — renders the 10 waypoints and edges as an RViz2 `MarkerArray` | No |
| [`vehicle_tf_broadcaster_node.cpp`](src/vehicle_tf_broadcaster_node.cpp) | Calls `solve_intra_factory_transport()` once on startup, then drives each vehicle's `DiffDrivePath`, broadcasting tf2, publishing body markers, and logging status every second | Yes (via linking `cuopt_solver`) |

```
┌───────────────────────┐  ┌──────────────────────────┐  ┌────────────────────────┐
│ waypoint_graph.cpp     │  │ cuopt_solver.cu            │  │ diff_drive_path.cpp     │
│ (ament_cmake, CXX)     │─►│ (ament_cmake, CXX+CUDA)    │─►│ (ament_cmake, CXX)      │
│                        │  │                            │  │                        │
│ 10 waypoints, 11 edges │  │ compute_cost_matrix        │  │ rotate / translate /    │
│ coords -> Euclid dist  │  │ solve() -- 6 pairs, 2 AMRs │  │ dwell primitives        │
└───────────────────────┘  │ compute_waypoint_sequence  │  └───────────┬────────────┘
                            └──────────────┬─────────────┘              │
                                           │ link -lcuopt -lrmm -lraft  ▼
                                           ▼                  ┌──────────────────────┐
                                    libcuopt.so                │ vehicle_tf_broadcaster│
                              (conda env: ~/miniconda3/        │ _node                 │
                               envs/cuopt)                     │ -> tf2 broadcast      │
                                                                │ -> RViz2 body markers │
                                                                │ -> per-second status  │
                                                                └──────────────────────┘
```

`waypoint_marker_publisher_node` sits off to the side of this pipeline entirely — it only reads `waypoint_graph.cpp`'s static node/edge data and renders it as a `MarkerArray`, with no CUDA dependency at all.

## Factory Floor Graph

10 waypoints, coordinates in meters (defined in [`waypoint_graph.cpp`](src/waypoint_graph.cpp)):

| id | coordinates | role |
|---|---|---|
| 0 | (-4, 0) | depot (fleet start/return point) |
| 1 | (-2, -2) | transit junction |
| 2 | (-2, 0) | transit junction |
| 3 | (-2, 2) | transit junction |
| 4 | (0, -2) | station A (pickup/delivery point) |
| 5 | (0, 0) | station B (pickup/delivery point) |
| 6 | (0, 2) | station C (pickup/delivery point) |
| 7 | (1, -2) | transit junction |
| 8 | (1, 0) | transit junction |
| 9 | (1, 2) | transit junction |

Edge connectivity (11 undirected edges; length is always computed on the fly from the two endpoints' coordinates):
`0-2, 1-2, 2-3, 1-4, 2-5, 3-6, 4-7, 5-8, 6-9, 7-8, 8-9`

Most edges are 2m, but after moving nodes 7/8/9 inward, `4-7`, `5-8`, and `6-9` are now 1m each. Stations A/B/C (4/5/6) are **not** directly connected to each other — reaching one from another always requires detouring through the outer-ring transit junctions (1/2/3 or 7/8/9).

## The Routing Problem

**In one sentence**: the depot dispatches 2 AMRs, each with capacity 2, to complete 6 pickup-delivery jobs among the factory floor's 3 stations (A/B/C) — one job in each direction between every pair of stations. Each job's pickup and delivery both carry a time window and a 10-second service time, and the goal is to find a feasible fleet/route assignment within the time limit that minimizes total cost (vehicle count first by default, then cost — see [How cuOpt Computes Cost](#how-cuopt-computes-cost)).

Every value below is exactly what `solve_intra_factory_transport()` in [`cuopt_solver.cu`](src/cuopt_solver.cu) feeds into `data_model_view_t`, named to match the variable in the code.

#### Graph-level input (feeds `waypoint_matrix_t`)

| Input | Source | Description |
|---|---|---|
| `h_offsets` / `h_indices` / `h_weights` | `build_csr()`, built dynamically from `waypoint_graph.cpp`'s `waypoints()`/`edges()` | A 10-node, 11-edge CSR graph; weight = the Euclidean distance between endpoints (8 edges are 2m, 3 edges — `4-7`/`5-8`/`6-9` — are 1m) |
| `target_locations` | `compute_target_locations()` | The compressed target-location list `{0, 4, 5, 6}` (depot, station A, B, C), which fixes the cost matrix's index order |
| `d_cost_matrix` | Computed by `waypoint_matrix_t::compute_cost_matrix(...)` — never filled in by hand | 4×4, in meters |
| `d_transit_time_matrix` | Computed by dividing every entry of `d_cost_matrix` by `kAssumedSpeedMps` | 4×4, in seconds; fed to `add_transit_time_matrix` — time-window feasibility is checked against this, not the cost matrix |

#### Fleet input (`fleet_size = 2`)

| Variable | Value | Description |
|---|---|---|
| `h_capacities` | `{2, 2}` | Capacity of each AMR (single dimension, treated as "number of items") |
| `h_vehicle_start` / `h_vehicle_end` | `{0, 0}` / `{0, 0}` | Both vehicles start from compressed index 0 (the depot) and return there |
| `h_vehicle_earliest` / `h_vehicle_latest` | `{0, 0}` / `{300, 300}` | Overall vehicle working time window of `[0, 300]` seconds |

#### Order input (`num_orders = 12`, 6 pickup-delivery pairs)

`h_order_locations` uses compressed indices (1=A, 2=B, 3=C); `h_pickup_orders`/`h_delivery_orders` bind the two order indices belonging to the same pair together; `h_demands` is +1 at pickup and -1 at delivery.

| pair | order idx (pickup→delivery) | route | pickup window | delivery window | service time |
|---|---|---|---|---|---|
| 1 | 0 → 1 | A → B | [0, 60] | [0, 120] | 10s |
| 2 | 2 → 3 | B → C | [0, 60] | [0, 120] | 10s |
| 3 | 4 → 5 | C → A | [0, 60] | [0, 120] | 10s |
| 4 | 6 → 7 | A → C | [20, 90] | [20, 150] | 10s |
| 5 | 8 → 9 | B → A | [20, 90] | [20, 150] | 10s |
| 6 | 10 → 11 | C → B | [20, 90] | [20, 150] | 10s |

```cpp
h_order_locations  = {1,2, 2,3, 3,1, 1,3, 2,1, 3,2};             // pickup, delivery interleaved
h_pickup_orders    = {0, 2, 4, 6, 8, 10};
h_delivery_orders  = {1, 3, 5, 7, 9, 11};
h_demands          = {1,-1, 1,-1, 1,-1, 1,-1, 1,-1, 1,-1};
h_order_earliest   = {0,0, 0,0, 0,0, 20,20, 20,20, 20,20};
h_order_latest     = {60,120, 60,120, 60,120, 90,150, 90,150, 90,150};
h_service_times    = all 12 entries are 10 (kServiceTimeSec)
```

`pair_id` (the number shown in `cargo=[...]` in the log) is exactly the leftmost column of this table — the code maps each pair's pickup and delivery order index back to it (see the cargo-tracking note under [Run](#run)).

#### Solver settings

| Setting | Value |
|---|---|
| `solver_settings_t::set_time_limit` | 5.0 seconds |
| `objective_function` | `set_objective_function` is never called, so the solver uses its default (vehicle count first, then cost) |

#### Output

Once solved, `host_assignment_t` returns a "target-location level" route (expressed in compressed indices); the code then uses `compute_waypoint_sequence` to expand it back into the full 10-node path — this step matters because getting from one station to another may require detouring through several transit junctions. On this machine (RTX 5070 Ti), the measured total cost is **40.0** with both vehicles used — lower than the 48.0 measured right after the A/B/C shortcuts were removed (see [How cuOpt Computes Cost](#how-cuopt-computes-cost)), since moving nodes 7/8/9 inward made the outer-ring detour cheaper again.

## How cuOpt Computes Cost

There are three distinct layers of "cost" here, which is easy to conflate — cross-checked directly against the installed `libcuopt` headers, not guessed.

**Edge weight (the lowest layer)** is simply the length of each edge in `waypoint_graph.cpp`, computed on the fly as the Euclidean distance between its two endpoints. This layer never touches cuOpt at all.

**The cost matrix** (`compute_cost_matrix`): once `cuopt::distance_engine::waypoint_matrix_t` has the 10-node graph, it uses a **shortest-path algorithm** (the private `dijkstra()` / `mpsp()` methods in `waypoint_matrix.hpp` — Dijkstra once per target location as the source, aggregated into an all-pairs result) to compute the shortest distance between any two target locations — **not** the straight-line distance between their coordinates. Its unit is meters.

> **An easy trap**: calling `add_cost_matrix` without also calling `add_transit_time_matrix` makes cuOpt check `order_earliest`/`order_latest` feasibility using that "meters" matrix directly as travel time — implicitly assuming "1 meter of travel = 1 second." [`cuopt_solver.cu`](src/cuopt_solver.cu) fixes this by computing a separate `d_transit_time_matrix = d_cost_matrix / kAssumedSpeedMps` (`kAssumedSpeedMps = 0.5`, matching `vehicle_tf_broadcaster_node`'s default `v_max`) via `thrust::transform` on the GPU, and feeding that into `add_transit_time_matrix` instead.

**What the solver actually minimizes**: `result.get_total_objective()` (the `total cost = 40.0` printed in the log) is a weighted combination of the criteria in `objective_t` (`routing_structures.hpp`):

```cpp
enum class objective_t {
  COST,                          // total distance/cost per the cost matrix
  TRAVEL_TIME,                   // total driving time per the travel time matrix (excludes waiting)
  VARIANCE_ROUTE_SIZE,           // variance in route length (stop count) across vehicles
  VARIANCE_ROUTE_SERVICE_TIME,   // variance in service time across vehicles
  PRIZE,                         // sum of prizes for orders that have a prize value and were served
  VEHICLE_FIXED_COST,            // fixed cost incurred for every additional vehicle used
  SIZE
};
```

`data_model_view_t::set_objective_function(objectives, weights, n)` lets you customize which criteria to use; the header comment spells out cuOpt's two-phase strategy:

> *"During improvement phase the solver only optimizes for the cost. This function is used to select the best solution across all climbers based on other criteria... The value for the VEHICLE criteria should be set to a high value (2^32)."*

cuOpt runs several "climbers" in parallel (independent local-search instances), each driving **COST** down during its improvement phase; once every climber finishes, the solver picks the one result it returns from the full weighted-criteria evaluation (default: vehicle count first, then cost — matching `pdptw_mixed_fleet_demo`'s behavior in `cuopt_routing_node`'s README). This package never calls `set_objective_function`, so it runs on that default.

**Solve time vs. accuracy** (`solver_settings_t::set_time_limit`): the header comment notes problems under 100 locations usually get reasonable accuracy within a second, and the default time limit is `num_locations / 5` seconds. Our problem has only 4 target locations, so the 5 seconds set in the code is well above the default.

## Getting Started

### Installation

1. **Install ROS2 Jazzy** (if not already installed)

   ```bash
   sudo apt install ros-jazzy-ros-base python3-colcon-common-extensions
   ```

2. **Install a CMake >= 3.30.4** (RMM/RAFT's CMake config files require it; Ubuntu 24.04's apt package is too old)

   ```bash
   python3 -m venv ~/.venvs/buildtools
   ~/.venvs/buildtools/bin/pip install "cmake>=3.30.4"
   export PATH="$HOME/.venvs/buildtools/bin:$PATH"
   cmake --version   # confirm >= 3.30.4
   ```

3. **Install the cuOpt SDK (headers + shared library) into a conda env** — identical to `cuopt_routing_node`'s requirement elsewhere in this workspace

   ```bash
   conda create -n cuopt python=3.12
   conda install -n cuopt -c rapidsai -c conda-forge -c nvidia \
     libcuopt libraft-headers librmm cuda-nvcc cuda-version=13
   ```

   Do **not** activate it. Just record the path:

   ```bash
   export CUOPT_SDK_PREFIX="$HOME/miniconda3/envs/cuopt"
   ```

### Build

```bash
cd ~/catkin_ws
source /opt/ros/jazzy/setup.bash
export CUOPT_SDK_PREFIX="$HOME/miniconda3/envs/cuopt"
export PATH="$HOME/.venvs/buildtools/bin:$PATH"

colcon build --packages-select cuopt_intra_factory_viz
source install/setup.bash
```

Verify conda is not active in the build shell:

```bash
echo "CONDA_PREFIX=$CONDA_PREFIX"   # must be empty
```

### Run

The easiest way to run everything together (visualization + simulated vehicles + RViz2):

```bash
ros2 launch cuopt_intra_factory_viz factory_demo.launch.py
```

Optional launch arguments map to the differential-drive kinematic limits:

```bash
ros2 launch cuopt_intra_factory_viz factory_demo.launch.py v_max:=0.8 a_lin:=0.5 w_max:=1.2 a_ang:=2.0 use_rviz:=false
```

#### `waypoint_marker_publisher_node`

Pure visualization, no CUDA/cuOpt call at all.

```bash
ros2 run cuopt_intra_factory_viz waypoint_marker_publisher_node
```

Publishes an RViz2 `MarkerArray` for the 10 waypoints and 11 edges described in [Factory Floor Graph](#factory-floor-graph).

#### `vehicle_tf_broadcaster_node`

Solves the routing problem once via `libcuopt`, then drives both AMRs around the expanded route and broadcasts tf2.

```bash
ros2 run cuopt_intra_factory_viz vehicle_tf_broadcaster_node
```
```
[vehicle_tf_broadcaster]: solving Intra-factory Transport scenario via cuOpt...
[vehicle_tf_broadcaster]: solved: total cost = 40.0, 2 vehicle(s) used
[vehicle_tf_broadcaster]: [amr0] pos=(-3.85, 0.00) heading=0.0 deg | TRANSLATING 0.15/4.00 m (0.30 m/s) -> heading to waypoint 5 | cargo=[]
[vehicle_tf_broadcaster]: [amr0] pos=(0.00, 0.00) | LOADING pair 4 at waypoint 5 (9.6/10.0 s) | cargo=[]
[vehicle_tf_broadcaster]: [amr0] pos=(0.00, 0.00) | LOADING pair 1 at waypoint 5 (0.6/10.0 s) | cargo=[4]
[vehicle_tf_broadcaster]: [amr0] pos=(0.00, 0.00) | LOADING pair 1 at waypoint 5 (1.6/10.0 s) | cargo=[4]
```
(verified on this GPU — `amr0` loads pair 4 at one position in the path, finishes, then **without moving** immediately loads pair 1, a second dwell at the exact same physical position; at that point it is already carrying 2 shipments at once, exactly at its capacity. The `amr<N>` numbering is whatever vehicle id cuOpt actually assigns, not hardcoded — which vehicles get used and what route each one drives can differ between solves.)

#### API gotchas found while building this (each verified against a real solve, not just read from docs)

1. **`compute_cost_matrix` alone is not enough for time windows.** Without a separate `add_transit_time_matrix` call, cuOpt reuses the "meters" cost matrix as travel time when checking `order_earliest`/`order_latest`, implicitly assuming 1 meter of travel equals 1 second. Fixed by computing `d_transit_time_matrix = d_cost_matrix / kAssumedSpeedMps` on the GPU and feeding that in separately.
2. **Dwell events must be keyed by position in the path, not by waypoint id.** A capacity-2 vehicle can pass through the same physical station twice in a row (serving two different orders) with zero travel in between — `DwellEvent::waypoint_index` addresses the exact spot in the expanded path, and `DiffDrivePath` had to be fixed to apply *every* dwell event at a given index, not just the first match, or 3 out of 4 stacked pickups at one station were silently dropped.
3. **A pickup and its matching delivery are two different `order_id`s in cuOpt**, which otherwise makes cargo-tracking logs read as two unrelated shipments (e.g. "loaded order 2" / "unloaded order 3"). Mapping both halves of `h_pickup_orders`/`h_delivery_orders` back to one shared `pair_id` fixes this.
4. **cuOpt's solve is not deterministic across runs.** Re-running `solve_intra_factory_transport()` on the exact same graph gave the same total cost (40.0) but a different vehicle/pair assignment and a different expanded path each time — consistent with the parallel-"climbers" strategy described above.

## Known Limitations

- `vehicle_tf_broadcaster_node` solves **synchronously, once, on startup** — it blocks for roughly 5 seconds (the solver time limit) before anything starts moving. This is intentional, in the same spirit as `cuopt_milp_node`'s GPU warmup; there is currently no service/action that lets you trigger a fresh solve on demand.
- There is **no real-time collision avoidance** between the two vehicles — it just happens that this particular problem's routes don't fight over the same stretch of track at the same time.
- The Dijkstra utility in `shortest_path.hpp/.cpp` is not currently used by the demo nodes (routing is now done by cuOpt instead); it's kept around because it's a small, self-contained, generally useful piece of code on its own.

## Reference

1. [NVIDIA cuOpt User Guide](https://docs.nvidia.com/cuopt/user-guide/latest/introduction.html)
2. [cuOpt Intra-factory Transport example (Python)](https://docs.nvidia.com/cuopt/user-guide/latest/cuopt-python/routing/routing-examples.html)
3. [`cuopt_routing_node`](../cuopt_routing_node) — this workspace's standalone, non-ROS-visualized solve of the same scenario (`intra_factory_transport_demo`), useful for comparing solver output in isolation
4. [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)

---

This repository is for your reference only. copying, patent application, academic journals are strictly prohibited.

Copyright © 2026 ZM Robotics Software Laboratory.