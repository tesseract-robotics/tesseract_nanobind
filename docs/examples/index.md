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
| `reeds_shepp_example.py` | `tesseract_reeds_shepp_example` | Reeds-Shepp path for differential-drive |

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
| `chain_example.py` | `tesseract_chain_example` | Descartes + TrajOpt chaining |

### Real-Time

| Example | Console script | Description |
|---------|----------------|-------------|
| `online_planning_example.py` | `tesseract_online_planning_example` | TaskComposer replanning |
| `online_planning_sqp_example.py` | `tesseract_online_planning_sqp_example` | Low-level SQP (73 Hz) |

### Visualization

| Example | Console script | Description |
|---------|----------------|-------------|
| `shapes_viewer.py` | `tesseract_shapes_viewer` | Basic shapes in viewer |
| `tesseract_material_mesh_viewer.py` | `tesseract_material_mesh_viewer` | Materials and meshes |
| `abb_irb2400_viewer.py` | `tesseract_abb_irb2400_viewer` | Robot visualization |
| `twc_workcell_positioner_viewer.py` | `tesseract_twc_workcell_positioner_viewer` | Industrial workcell (positioner, requires submodule) |

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

viewer = TesseractViewer()
viewer.update_environment(robot.env, [0, 0, 0])

# Animate trajectory
viewer.update_trajectory(result.raw_results)

# Start web server
viewer.start_serve_background()
print("Open http://localhost:8000")
input("Press Enter to exit...")
```

Open `http://localhost:8000` in your browser to view.
