# Examples

Every example is a script that runs on its own: installed as a console script, importable as a
module, and tested in CI. The recorded ones play in the browser on their page: click a poster,
then orbit, scrub and change the playback speed, with nothing installed.

```bash
tesseract_car_seat_example                              # an installed console script
python -m tesseract_robotics.examples.car_seat_example  # the same, as a module
```

Run as a script, an example plans, opens the [viewer](../user-guide/viewer.md) and prints its URL.
Its `run()` function plans without the viewer and returns the results; the tests call that. A
plan that fails raises `PlanningFailedError` or `TimeParameterizationFailedError`
(`tesseract_robotics.examples.failures`) and the script exits non-zero.

## Planning

Cartesian toolpaths, orientation constraints, attach-and-place and multi-step programs.

<div class="grid cards" markdown>

-   [![Car seat demo, opening frame](../demos/posters/car_seat_example.png)](planning.md#multi-step-with-task-composer-car-seat)

    **[Car seat](planning.md#multi-step-with-task-composer-car-seat)** · [source](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/car_seat_example.py)

    Picks a seat off a conveyor and places it through a car's door opening, on a rail-mounted robot.

    `tesseract_car_seat_example`

-   [![Pick and place demo, opening frame](../demos/posters/pick_and_place_example.png)](planning.md#pick-and-place)

    **[Pick and place](planning.md#pick-and-place)** · [source](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/pick_and_place_example.py)

    Picks a box, attaches it to the tool and places it on a shelf.

    `tesseract_pick_and_place_example`

-   [![Raster demo, opening frame](../demos/posters/raster_example.png)](planning.md#raster-pattern)

    **[Raster](planning.md#raster-pattern)** · [source](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/raster_example.py)

    An industrial raster pattern, as used for welding or milling.

    `tesseract_raster_example`

-   [![Puzzle piece demo, opening frame](../demos/posters/puzzle_piece_auxillary_axes_example.png)](planning.md#9-dof-with-external-axis-puzzle-piece)

    **[Puzzle piece](planning.md#9-dof-with-external-axis-puzzle-piece)** · [source](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/puzzle_piece_auxillary_axes_example.py)

    A 9-DOF Cartesian toolpath: a robot and two auxiliary axes grind a part's edge.

    `tesseract_puzzle_piece_auxillary_axes_example`

-   [![Chained raster demo, opening frame](../demos/posters/chain_example.png)](planning.md#descartes-trajopt-chaining)

    **[Chained raster](planning.md#descartes-trajopt-chaining)** · [source](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/chain_example.py)

    A raster planned with Descartes and refined with TrajOpt.

    `tesseract_chain_example`

-   [![Glass upright demo, opening frame](../demos/posters/glass_upright_example.png)](planning.md#glass-upright-orientation-constraint)

    **[Glass upright](planning.md#glass-upright-orientation-constraint)** · [source](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/glass_upright_example.py)

    Moves a tool while an orientation constraint keeps it upright.

    `tesseract_glass_upright_example`

-   [![Basic Cartesian demo, opening frame](../demos/posters/basic_cartesian_example.png)](planning.md#basic-cartesian)

    **[Basic Cartesian](planning.md#basic-cartesian)** · [source](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/basic_cartesian_example.py)

    A Cartesian path planned with TrajOpt.

    `tesseract_basic_cartesian_example`

</div>

## Sampling-based and hybrid

OMPL alone, and OMPL followed by TrajOpt.

<div class="grid cards" markdown>

-   [![Freespace OMPL demo, opening frame](../demos/posters/freespace_ompl_example.png)](ompl.md#freespace-ompl)

    **[Freespace OMPL](ompl.md#freespace-ompl)** · [source](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/freespace_ompl_example.py)

    Freespace motion around an obstacle, planned with OMPL.

    `tesseract_freespace_ompl_example`

-   [![Freespace hybrid demo, opening frame](../demos/posters/freespace_hybrid_example.png)](ompl.md#hybrid-planning-ompl-trajopt)

    **[Freespace hybrid](ompl.md#hybrid-planning-ompl-trajopt)** · [source](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/freespace_hybrid_example.py)

    OMPL finds a path around an obstacle; TrajOpt smooths it.

    `tesseract_freespace_hybrid_example`

</div>

## Online planning

Replanning while the world changes.

<div class="grid cards" markdown>

-   [![Online planning demo, opening frame](../demos/posters/online_planning_example.png)](online-planning.md#overview)

    **[Online planning](online-planning.md#overview)** · [source](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/online_planning_example.py)

    Replans the gantry's run while a person walks up to the arm and back.

    `tesseract_online_planning_example`

-   [![Online planning, SQP demo, opening frame](../demos/posters/online_planning_sqp_example.png)](online-planning.md#low-level-sqp-example)

    **[Online planning, SQP](online-planning.md#low-level-sqp-example)** · [source](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/online_planning_sqp_example.py)

    Replanning through the low-level SQP API, one warm-started solve per tick.

    `tesseract_online_planning_sqp_example`

-   [![Python constraints demo, opening frame](../demos/posters/python_constraints_example.png)](python-constraints.md#what-you-are-watching)

    **[Python constraints](python-constraints.md#what-you-are-watching)** · [source](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/python_constraints_example.py)

    Seven rules written in Python, switched on one after another in a live loop: one solve per frame, a gauge per rule, the solve time against the frame.

    `tesseract_python_constraints_example`

</div>

## Collision and scenes

Point clouds, signed distance fields, workcells, scene graphs and geometry.

<div class="grid cards" markdown>

-   [![Point cloud collision demo, opening frame](../demos/posters/pointcloud_octree_collision_example.png)](basic.md#point-cloud-octree-example)

    **[Point cloud collision](basic.md#point-cloud-octree-example)** · [source](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/pointcloud_octree_collision_example.py)

    Plans around an obstacle known only as a point cloud, through an octree.

    `tesseract_pointcloud_octree_collision_example`

-   [![SDF collision demo, opening frame](../demos/posters/sdf_collision_example.png)](collision.md#signed-distance-field)

    **[SDF collision](collision.md#signed-distance-field)** · [source](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/sdf_collision_example.py)

    A probe swept through a signed distance field.

    `tesseract_sdf_collision_example`

-   [![TWC workcell demo, opening frame](../demos/posters/twc_workcell_positioner_viewer.png)](../user-guide/viewer.md#industrial-workcells)

    **[TWC workcell](../user-guide/viewer.md#industrial-workcells)** · [source](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/twc_workcell_positioner_viewer.py)

    An industrial workcell: a robot on a 12 m rail, beside a positioner.

    `tesseract_twc_workcell_positioner_viewer`

-   [![ABB IRB2400 demo, opening frame](../demos/posters/abb_irb2400_viewer.png)](planning.md#visualization)

    **[ABB IRB2400](planning.md#visualization)** · [source](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/abb_irb2400_viewer.py)

    Plans a move on the ABB IRB2400 and draws its TCP path.

    `tesseract_abb_irb2400_viewer`

-   [![Scene graph demo, opening frame](../demos/posters/scene_graph_example.png)](basic.md#scene-graph-example)

    **[Scene graph](basic.md#scene-graph-example)** · [source](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/scene_graph_example.py)

    A robot built from a scene graph.

    `tesseract_scene_graph_example`

-   [![Geometry showcase demo, opening frame](../demos/posters/geometry_showcase_example.png)](basic.md#geometry-showcase)

    **[Geometry showcase](basic.md#geometry-showcase)** · [source](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/geometry_showcase_example.py)

    Every geometry type the environment holds: box, sphere, cylinder, mesh and more.

    `tesseract_geometry_showcase_example`

-   [![Shapes demo, opening frame](../demos/posters/shapes_viewer.png)](geometry.md#shapes)

    **[Shapes](geometry.md#shapes)** · [source](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/shapes_viewer.py)

    Primitive shapes in the viewer.

    `tesseract_shapes_viewer`

-   [![Material mesh demo, opening frame](../demos/posters/tesseract_material_mesh_viewer.png)](geometry.md#material-mesh)

    **[Material mesh](geometry.md#material-mesh)** · [source](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/tesseract_material_mesh_viewer.py)

    A mesh with materials and textures.

    `tesseract_material_mesh_viewer`

</div>

## Every example

The console script is the module name with a `tesseract_` prefix (kept when the module already
has it). The page column links the walkthrough.

| Module | Console script | Page | What it shows |
| --- | --- | --- | --- |
| [`basic_cartesian_example`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/basic_cartesian_example.py) | `tesseract_basic_cartesian_example` | [Planning](planning.md) | Cartesian path with TrajOpt |
| [`glass_upright_example`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/glass_upright_example.py) | `tesseract_glass_upright_example` | [Planning](planning.md) | Linear move with an orientation constraint |
| [`pick_and_place_example`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/pick_and_place_example.py) | `tesseract_pick_and_place_example` | [Planning](planning.md) | Pick, attach, place |
| [`raster_example`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/raster_example.py) | `tesseract_raster_example` | [Planning](planning.md) | Raster toolpath with freespace transitions |
| [`car_seat_example`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/car_seat_example.py) | `tesseract_car_seat_example` | [Planning](planning.md) | Multi-step program on a rail-mounted robot |
| [`chain_example`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/chain_example.py) | `tesseract_chain_example` | [Planning](planning.md) | Descartes raster refined with TrajOpt |
| [`puzzle_piece_auxillary_axes_example`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/puzzle_piece_auxillary_axes_example.py) | `tesseract_puzzle_piece_auxillary_axes_example` | [Planning](planning.md) | 9-DOF Cartesian toolpath: a 7-DOF arm and a 2-DOF positioner |
| [`abb_irb2400_viewer`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/abb_irb2400_viewer.py) | `tesseract_abb_irb2400_viewer` | [Planning](planning.md) | A planned move and its TCP path in the viewer |
| [`pythonic_api_example`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/pythonic_api_example.py) | `tesseract_pythonic_api_example` | [Low-level examples](#low-level-examples) | The low-level examples' steps through `tesseract_robotics.planning` |
| [`freespace_ompl_example`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/freespace_ompl_example.py) | `tesseract_freespace_ompl_example` | [OMPL](ompl.md) | OMPL freespace motion |
| [`freespace_hybrid_example`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/freespace_hybrid_example.py) | `tesseract_freespace_hybrid_example` | [OMPL](ompl.md) | OMPL, then TrajOpt |
| [`freespace_clearance_objective_example`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/freespace_clearance_objective_example.py) | `tesseract_freespace_clearance_objective_example` | [OMPL](ompl.md) | OMPL objective written in Python; a cost threshold ends the optimize loop |
| [`online_planning_example`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/online_planning_example.py) | `tesseract_online_planning_example` | [Online planning](online-planning.md) | Task composer replanning around a walking person |
| [`online_planning_sqp_example`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/online_planning_sqp_example.py) | `tesseract_online_planning_sqp_example` | [Online planning](online-planning.md) | Low-level SQP replanning, warm-started each tick |
| [`python_constraints_example`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/python_constraints_example.py) | `tesseract_python_constraints_example` | [Python constraints](python-constraints.md) | Seven constraints written in Python, in a live loop, scripted or driven by sliders |
| [`tesseract_kinematics_example`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/tesseract_kinematics_example.py) | `tesseract_kinematics_example` | [Basics](basic.md) | FK and IK with a kinematic group |
| [`tesseract_collision_example`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/tesseract_collision_example.py) | `tesseract_collision_example` | [Basics](basic.md) | Discrete collision check |
| [`scene_graph_example`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/scene_graph_example.py) | `tesseract_scene_graph_example` | [Basics](basic.md) | Build and query a scene graph |
| [`geometry_showcase_example`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/geometry_showcase_example.py) | `tesseract_geometry_showcase_example` | [Basics](basic.md) | Every geometry type |
| [`pointcloud_octree_collision_example`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/pointcloud_octree_collision_example.py) | `tesseract_pointcloud_octree_collision_example` | [Basics](basic.md) | Point cloud to octree to collision-aware TrajOpt |
| [`kinematics_plugins_example`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/kinematics_plugins_example.py) | `tesseract_kinematics_plugins_example` | [Kinematics](kinematics.md) | Kinematics plugin factory: solvers, configs |
| [`kinematics_analysis_example`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/kinematics_analysis_example.py) | `tesseract_kinematics_analysis_example` | [Kinematics](kinematics.md) | Jacobians, twists, manipulability, limits |
| [`state_solver_example`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/state_solver_example.py) | `tesseract_state_solver_example` | [Kinematics](kinematics.md) | State solver: floating joints, Jacobians |
| [`contact_managers_example`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/contact_managers_example.py) | `tesseract_contact_managers_example` | [Collision](collision.md) | Contact managers, margins, ACM, cast manager |
| [`trajectory_collision_example`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/trajectory_collision_example.py) | `tesseract_trajectory_collision_example` | [Collision](collision.md) | Trajectory checks, LVS, rebuilt from their parts |
| [`convex_decomposition_example`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/convex_decomposition_example.py) | `tesseract_convex_decomposition_example` | [Collision](collision.md) | Convex decomposition options and fill modes |
| [`sdf_collision_example`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/sdf_collision_example.py) | `tesseract_sdf_collision_example` | [Collision](collision.md) | A probe swept through a signed distance field |
| [`environment_commands_example`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/environment_commands_example.py) | `tesseract_environment_commands_example` | [Environment](environment.md) | Commands, history, replay, calibration |
| [`environment_state_example`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/environment_state_example.py) | `tesseract_environment_state_example` | [Environment](environment.md) | State, floating joints, TCPs, manipulator info |
| [`plugin_configuration_example`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/plugin_configuration_example.py) | `tesseract_plugin_configuration_example` | [Environment](environment.md) | Contact manager and task composer plugin configuration |
| [`mesh_roundtrip_example`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/mesh_roundtrip_example.py) | `tesseract_mesh_roundtrip_example` | [Geometry](geometry.md) | Meshes from bytes, materials, round trips |
| [`shapes_viewer`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/shapes_viewer.py) | `tesseract_shapes_viewer` | [Geometry](geometry.md) | Primitive shapes in the viewer |
| [`tesseract_material_mesh_viewer`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/tesseract_material_mesh_viewer.py) | `tesseract_material_mesh_viewer` | [Geometry](geometry.md) | Materials and textures |
| [`twc_workcell_positioner_viewer`](https://github.com/tesseract-robotics/tesseract_nanobind/blob/main/src/tesseract_robotics/examples/twc_workcell_positioner_viewer.py) | `tesseract_twc_workcell_positioner_viewer` | [Viewer](../user-guide/viewer.md) | Industrial workcell with a positioner (needs `git submodule update --init`) |

## Low-level examples

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
