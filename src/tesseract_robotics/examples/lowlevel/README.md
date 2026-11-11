# Low-Level Examples

Direct C++ binding examples for tesseract_robotics.

These examples demonstrate the raw nanobind API - a 1:1 mapping of the C++ Tesseract libraries. They use the bindings only, never `tesseract_robotics.planning` (`tests/examples/test_lowlevel_bindings_only.py` enforces it), and the ports of the C++ `tesseract_planning/examples` keep their exact parameters. `example_support.py` holds the plumbing the C++ examples share (`example.cpp`): loading a `tesseract_support` robot, adding a fixed obstacle link, running a task composer pipeline. Use these as reference for understanding the underlying mechanics or when you need fine-grained control not exposed by the high-level API.

**For most use cases, prefer the [high-level examples](../) instead.**

## Examples

| Example | Description |
|---------|-------------|
| `freespace_ompl_c_api_example.py` | OMPL planning with manual environment setup |
| `basic_cartesian_c_api_example.py` | TrajOpt Cartesian planning |
| `glass_upright_c_api_example.py` | Constrained planning with profiles |
| `puzzle_piece_c_api_example.py` | Toolpath from CSV with TrajOpt |
| `puzzle_piece_auxillary_axes_c_api_example.py` | 9-DOF planning |
| `pick_and_place_c_api_example.py` | Attach/detach collision objects |
| `car_seat_c_api_example.py` | Multi-phase planning with meshes |
| `scene_graph_c_api_example.py` | Scene graph manipulation |
| `tesseract_collision_c_api_example.py` | Collision manager API |
| `tesseract_kinematics_c_api_example.py` | Kinematics groups |
| `tesseract_planning_lowlevel_c_api_example.py` | Planning without TaskComposer |
| `tesseract_planning_composer_c_api_example.py` | TaskComposer usage |
| `tesseract_planning_lowlevel_trajopt_ifopt_example.py` | OMPL then TrajOptIfopt (OSQP), planning without TaskComposer |
| `descartes_raster_c_api_example.py` | Descartes raster + TrajOpt refinement (port of C++ raster_example.cpp) |

## When to Use Low-Level API

- Implementing custom planners
- Accessing features not yet wrapped in high-level API
- Porting C++ code directly
- Debugging planning issues at the binding level
