# Examples

Learn tesseract_robotics through working examples.

## Example Categories

<div class="grid cards" markdown>

-   :material-robot-industrial:{ .lg .middle } **Basic Examples**

    ---

    Fundamentals: kinematics, collision, scene graph

    [:octicons-arrow-right-24: Basic Examples](basic.md)

-   :material-map-marker-path:{ .lg .middle } **Planning Examples**

    ---

    Motion planning with OMPL, TrajOpt, Descartes

    [:octicons-arrow-right-24: Planning Examples](planning.md)

-   :material-clock-fast:{ .lg .middle } **Online Planning**

    ---

    Real-time replanning with low-level SQP API

    [:octicons-arrow-right-24: Online Planning](online-planning.md)

</div>

## Running Examples

Examples install as console scripts via `[project.scripts]` in `pyproject.toml`.
Once the package is installed (via `pip` or `pixi install`) they are on `PATH`:

```bash
# Activate environment
pixi shell

# Run an example (installed console script)
tesseract_freespace_ompl_example

# Or via Python module invocation
pixi run python -m tesseract_robotics.examples.freespace_ompl_example
```

Every example module has both `main()` and `run()` entry points — `main()`
handles optional viewer launch, `run()` returns results for testing / reuse.

## Example Index

### Basic

| Example | Console script | Description |
|---------|----------------|-------------|
| `tesseract_kinematics_example.py` | `tesseract_kinematics_example` | FK/IK with joint groups |
| `tesseract_collision_example.py` | `tesseract_collision_example` | Discrete collision checking |
| `scene_graph_example.py` | `tesseract_scene_graph_example` | Scene graph manipulation |
| `geometry_showcase_example.py` | `tesseract_geometry_showcase_example` | All geometry types |
| `pointcloud_octree_collision_example.py` | `tesseract_pointcloud_octree_collision_example` | Point cloud → Octree → TrajOpt with collision on/off, toggleable in the viewer |

### Planning

| Example | Console script | Description |
|---------|----------------|-------------|
| `freespace_ompl_example.py` | `tesseract_freespace_ompl_example` | OMPL freespace motion |
| `basic_cartesian_example.py` | `tesseract_basic_cartesian_example` | Cartesian path with TrajOpt |
| `glass_upright_example.py` | `tesseract_glass_upright_example` | Orientation constraints |
| `pick_and_place_example.py` | `tesseract_pick_and_place_example` | Pick, attach, place workflow |
| `car_seat_example.py` | `tesseract_car_seat_example` | Complex multi-step planning |
| `raster_example.py` | `tesseract_raster_example` | Industrial raster patterns |
| `puzzle_piece_auxillary_axes_example.py` | `tesseract_puzzle_piece_auxillary_axes_example` | 9-DOF with external axis |
| `freespace_hybrid_example.py` | `tesseract_freespace_hybrid_example` | OMPL + TrajOpt hybrid |
| `freespace_clearance_objective_example.py` | `tesseract_freespace_clearance_objective_example` | OMPL objective from Python; a cost threshold ends the optimize loop |
| `chain_example.py` | `tesseract_chain_example` | Descartes + TrajOpt chaining |

### Real-Time

| Example | Console script | Description |
|---------|----------------|-------------|
| `online_planning_example.py` | `tesseract_online_planning_example` | TaskComposer replanning |
| `online_planning_sqp_example.py` | `tesseract_online_planning_sqp_example` | Low-level SQP replanning ([rates](online-planning.md#performance-comparison)) |

### Visualization

| Example | Console script | Description |
|---------|----------------|-------------|
| `shapes_viewer.py` | `tesseract_shapes_viewer` | Basic shapes in viewer |
| `tesseract_material_mesh_viewer.py` | `tesseract_material_mesh_viewer` | Materials and meshes |
| `abb_irb2400_viewer.py` | `tesseract_abb_irb2400_viewer` | Robot visualization |
| `twc_workcell_positioner_viewer.py` | `tesseract_twc_workcell_positioner_viewer` | Industrial workcell (positioner, requires submodule) |

### Low-level (bindings only)

`tesseract_robotics.examples.lowlevel` ports the C++ `tesseract_planning/examples` with
the binding types alone, never `tesseract_robotics.planning`, and with the C++ parameters
(`tests/examples/test_lowlevel_bindings_only.py` enforces the first). They are the
reference for what the high-level examples above wrap. Run them as modules:

```bash
python -m tesseract_robotics.examples.lowlevel.freespace_ompl_c_api_example
```

| Example | C++ original | Description |
|---------|--------------|-------------|
| `freespace_ompl_c_api_example.py` | `freespace_ompl_example.cpp` | `OMPLPipeline` around a sphere |
| `basic_cartesian_c_api_example.py` | `basic_cartesian_example.cpp` | TrajOpt Cartesian path past an octomap (`ifopt=True` for TrajOptIfopt) |
| `glass_upright_c_api_example.py` | `glass_upright_example.cpp` | Orientation-held linear move |
| `puzzle_piece_c_api_example.py` | `puzzle_piece_example.cpp` | Toolpath from CSV |
| `puzzle_piece_auxillary_axes_c_api_example.py` | `puzzle_piece_auxillary_axes_example.cpp` | 9-DOF toolpath with a positioner |
| `pick_and_place_c_api_example.py` | `pick_and_place_example.cpp` | Pick, attach, place |
| `car_seat_c_api_example.py` | `car_seat_example.cpp` | Seat meshes, attach, multi-phase planning |
| `descartes_raster_c_api_example.py` | `motion_planners/examples/raster_example.cpp` | Descartes then TrajOpt, no task composer |
| `tesseract_planning_composer_c_api_example.py` | — | The task composer steps, one by one |
| `tesseract_planning_lowlevel_c_api_example.py` | — | Planners without the task composer |

`pythonic_api_example.py` (console module `tesseract_robotics.examples.pythonic_api_example`)
shows the same steps through `tesseract_robotics.planning`.

## Prerequisites

Most examples require:

```python
# tesseract_robotics installed
import tesseract_robotics

# For visualization
from tesseract_robotics.viewer import TesseractViewer
```

Install the package:

```bash
pip install tesseract-robotics-nanobind
```

## Viewer Usage

Most examples include visualization:

```python
from tesseract_robotics.viewer import TesseractViewer

with TesseractViewer() as viewer:  # serving from here until the block ends
    viewer.update_environment(robot.env)
    viewer.update_trajectory(result.raw_results)
    print(f"Viewer at {viewer.url}")
    input("Press Enter to exit...")
```

Open the printed URL in your browser; see the [viewer guide](../user-guide/viewer.md).
