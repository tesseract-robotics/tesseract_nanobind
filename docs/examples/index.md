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

All examples are in the `examples/` directory:

```bash
# Activate environment
pixi shell

# Run an example
python examples/freespace_ompl_example.py
```

## Example Index

### Basic

| Example | Description |
|---------|-------------|
| `tesseract_kinematics_example.py` | FK/IK with joint groups |
| `tesseract_collision_example.py` | Discrete collision checking |
| `scene_graph_example.py` | Scene graph manipulation |
| `geometry_showcase_example.py` | All geometry types |

### Planning

| Example | Description |
|---------|-------------|
| `freespace_ompl_example.py` | OMPL freespace motion |
| `basic_cartesian_example.py` | Cartesian path with TrajOpt |
| `glass_upright_example.py` | Orientation constraints |
| `pick_and_place_example.py` | Pick, attach, place workflow |
| `car_seat_example.py` | Complex multi-step planning |
| `raster_example.py` | Industrial raster patterns |
| `puzzle_piece_auxillary_axes_example.py` | 9-DOF with external axis |
| `freespace_hybrid_example.py` | OMPL + TrajOpt hybrid |
| `chain_example.py` | Descartes + TrajOpt chaining |

### Real-Time

| Example | Description |
|---------|-------------|
| `online_planning_example.py` | TaskComposer replanning |
| `online_planning_sqp_example.py` | Low-level SQP (73 Hz) |

### Visualization

| Example | Description |
|---------|-------------|
| `shapes_viewer.py` | Basic shapes in viewer |
| `tesseract_material_mesh_viewer.py` | Materials and meshes |
| `abb_irb2400_viewer.py` | Robot visualization |

## Prerequisites

Most examples require:

```python
# tesseract_robotics installed
import tesseract_robotics

# For visualization
from tesseract_robotics_viewer import TesseractViewer
```

Install the viewer:

```bash
pip install tesseract_robotics_viewer
```

## Viewer Usage

Most examples include visualization:

```python
from tesseract_robotics_viewer import TesseractViewer

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
