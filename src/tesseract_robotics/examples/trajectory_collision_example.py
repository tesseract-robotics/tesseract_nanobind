"""
Trajectory Collision Example
============================

Collision checks of single states, swept segments and whole trajectories, and a
trajectory turned into a collision link with `AddTrajectoryLinkCommand`.

C++ Reference:
    environment/src/utils.cpp:237-296  (checkTrajectory, LVS_CONTINUOUS branch)
    environment/test/tesseract_environment_utils.cpp:112-230  (boxbot state and segment checks)
    environment/test/tesseract_environment_unit.cpp:2905-2997, 5477-5493  (checkTrajectoryUnit)
    environment/test/tesseract_environment_unit.cpp:831-929  (EnvAddandRemoveTrajectoryLink)
    environment/include/tesseract/environment/commands/add_trajectory_link_command.h:52-73
    support/urdf/boxbot.urdf

Overview
--------
1. boxbot: two unit boxes 0.1 m in collision, checked with both
   `checkTrajectoryState` overloads (discrete and continuous manager) and a
   swept `checkTrajectorySegment`; disabling `boxbot_link` through the
   `ContactManagerConfig` clears each check.
2. IIWA with upstream's attached sphere: `checkTrajectory` with
   `LVS_CONTINUOUS` on upstream's 5-state trajectory, rebuilt step by step from
   `checkTrajectorySegment` over `LinSpaced` sub-states and
   `addInterpolatedCollisionResults`, exactly as `utils.cpp` does.
3. `JointTrajectory`: both constructors, `push_back`, `states`, `description`,
   `uuid`, indexing, iteration, `clear` / `empty`.
4. `AddTrajectoryLinkCommand` once per `Method`, replacing the same link; the
   link's collision count follows from each method's documented semantics.

Key Concepts
------------
**Penetration depth of the boxbot boxes**:
    The two unit boxes overlap by 0.1 m along x with their y and z faces
    coplanar. FCL reports the analytic -0.1 m. Bullet, the default discrete and
    the only continuous backend, reports -0.0577 m (0.1 / sqrt(3)) along a
    corner direction. Upstream's boxbot test asserts only that the pair is in
    contact, so the example asserts the depth on FCL and the pair on Bullet.

**Reusing a ContactResultMap**:
    `clear()` keeps every key, with an empty vector, and `==` compares keys.
    `checkTrajectory` reuses one map across steps (`utils.cpp:234`), so a pair
    that touched in an earlier step stays as an empty entry. The rebuild reuses
    one map the same way, which makes the per-step results compare equal.

**Trajectory link collision counts**:
    The active links are the children of the trajectory's joints that carry
    collision geometry. With `n` states and `c` collision shapes on `a` such
    links: `PER_STATE_OBJECTS` adds `n * c` shapes, `PER_STATE_CONVEX_HULL`
    one hull per state (`n`), `GLOBAL_PER_LINK_CONVEX_HULL` one hull per link
    (`a`) and `GLOBAL_CONVEX_HULL` one hull (`environment.cpp:1412-1471`).
"""

import math
import uuid
from pathlib import Path
from typing import Any

import numpy as np

from tesseract_robotics import get_tesseract_resource_path
from tesseract_robotics.collision import (
    CollisionCheckConfig,
    CollisionCheckExitType,
    CollisionCheckProgramType,
    CollisionEvaluatorType,
    ContactManagerConfig,
    ContactRequest,
    ContactResultMap,
    ContactTestType,
)
from tesseract_robotics.common import (
    GeneralResourceLocator,
    Isometry3d,
    JointState,
    JointTrajectory,
)
from tesseract_robotics.environment import (
    AddLinkCommand,
    AddTrajectoryLinkCommand,
    Environment,
    checkTrajectory,
    checkTrajectorySegment,
    checkTrajectoryState,
)
from tesseract_robotics.geometry import Sphere
from tesseract_robotics.scene_graph import Collision, Joint, JointType, Link, Visual

BOXBOT_URDF_URL = "package://tesseract/support/urdf/boxbot.urdf"
BOXBOT_SRDF_URL = "package://tesseract/support/urdf/boxbot.srdf"
IIWA_URDF_URL = "package://tesseract/support/urdf/lbr_iiwa_14_r820.urdf"
IIWA_SRDF_URL = "package://tesseract/support/urdf/lbr_iiwa_14_r820.srdf"

BOX_PAIR = ("boxbot_link", "test_box_link")
# tesseract_environment_utils.cpp:131-133, 184-190: test_box_link poses, m
STATE_OFFSET = (0.9, 0.0, 0.0)
SEGMENT_START = (0.9, 2.0, 0.0)
SEGMENT_END = (0.9, -2.0, 0.0)
# Unit boxes 0.9 m apart overlap by 1 - 0.9, m
PENETRATION = 1.0 - STATE_OFFSET[0]
# Distance tolerance of upstream's box-box unit tests, m (collision_box_box_unit.hpp:162, 243, 280)
BOX_BOX_TOL = 0.001

# checkTrajectoryUnit (tesseract_environment_unit.cpp:2911-2968): attached sphere and states, rad
SPHERE_RADIUS = 0.15
SPHERE_ORIGIN = (0.5, 0.0, 0.55)
IIWA_JOINTS = [f"joint_a{i}" for i in range(1, 8)]
JOINT_START = [-0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0]
JOINT_END = [0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0]
NUM_STATES = 5

# EnvAddandRemoveTrajectoryLink (tesseract_environment_unit.cpp:835-841)
TRAJECTORY_JOINTS = ["joint_a1", "joint_a2"]
TRAJECTORY_LINK = "traj_link"
TRAJECTORY_PARENT = "base_link"
TRAJECTORY_DESCRIPTION = "joint_a1 and joint_a2 from 0 to 1 rad"


def locate(locator: GeneralResourceLocator, url: str) -> Path:
    """File path of a `package://` resource; raises if the locator can't resolve it."""
    resource = locator.locateResource(url)
    if resource is None:
        raise FileNotFoundError(url)
    return Path(resource.getFilePath())


def translation(x: float, y: float = 0.0, z: float = 0.0) -> Isometry3d:
    """A pure translation."""
    matrix = np.eye(4)
    matrix[:3, 3] = [x, y, z]
    return Isometry3d(matrix)


def load(locator: GeneralResourceLocator, urdf_url: str, srdf_url: str) -> Environment:
    """An environment from a URDF and SRDF; raises if it does not initialise."""
    env = Environment()
    if not env.init(locate(locator, urdf_url), locate(locator, srdf_url), locator):
        raise RuntimeError(f"environment failed to initialise from {urdf_url}")
    return env


def contact_pairs(results: ContactResultMap) -> list[tuple[str, str]]:
    """Sorted link-name pairs that hold at least one contact."""
    return sorted(key.orderedNameView() for key, vector in results if len(vector))


def boxbot_checks(locator: GeneralResourceLocator) -> dict[str, Any]:
    """tesseract_environment_utils.cpp: boxbot state and segment checks at margin 0."""
    env = load(locator, BOXBOT_URDF_URL, BOXBOT_SRDF_URL)
    request = ContactRequest(ContactTestType.FIRST)
    config = ContactManagerConfig(0.0)
    disabled = ContactManagerConfig(0.0)
    disabled.modify_object_enabled = {"boxbot_link": False}

    # --8<-- [start:state]
    state = {"boxbot_link": translation(0), "test_box_link": translation(*STATE_OFFSET)}

    discrete = env.getDiscreteContactManager()
    discrete.setActiveCollisionObjects(list(BOX_PAIR))
    discrete.applyContactManagerConfig(config)
    discrete_contacts = checkTrajectoryState(discrete, state, request)

    continuous = env.getContinuousContactManager()
    continuous.setActiveCollisionObjects(list(BOX_PAIR))
    continuous.applyContactManagerConfig(config)
    continuous_contacts = checkTrajectoryState(continuous, state, request)
    # --8<-- [end:state]

    # --8<-- [start:segment]
    state0 = {"boxbot_link": translation(0), "test_box_link": translation(*SEGMENT_START)}
    state1 = {"boxbot_link": translation(0), "test_box_link": translation(*SEGMENT_END)}
    segment_contacts = checkTrajectorySegment(continuous, state0, state1, request)
    # --8<-- [end:segment]

    # --8<-- [start:state_fcl]
    fcl = env.getDiscreteContactManager("FCLDiscreteBVHManager")
    fcl.setActiveCollisionObjects(list(BOX_PAIR))
    fcl.applyContactManagerConfig(config)
    fcl_contacts = checkTrajectoryState(fcl, state, request)
    # --8<-- [end:state_fcl]

    result: dict[str, Any] = {"segment_pairs": contact_pairs(segment_contacts)}
    for kind, manager, contacts in [
        ("discrete", discrete, discrete_contacts),
        ("continuous", continuous, continuous_contacts),
        ("fcl", fcl, fcl_contacts),
    ]:
        manager.applyContactManagerConfig(disabled)
        result[kind] = {
            "pairs": contact_pairs(contacts),
            "distance": contacts.at(BOX_PAIR)[0].distance,
            "pairs_with_boxbot_disabled": contact_pairs(
                checkTrajectoryState(manager, state, request)
            ),
        }
    return result


def add_attached_sphere(env: Environment) -> None:
    """checkTrajectoryUnit: a 0.15 m sphere fixed to base_link."""
    origin = translation(*SPHERE_ORIGIN)
    visual = Visual()
    visual.origin = origin
    visual.geometry = Sphere(SPHERE_RADIUS)
    collision = Collision()
    collision.origin = origin
    collision.geometry = visual.geometry
    link = Link("sphere_attached")
    link.addVisual(visual)
    link.addCollision(collision)

    joint = Joint("joint_sphere_attached")
    joint.parent_link_id = "base_link"
    joint.child_link_id = link.getName()
    joint.type = JointType.FIXED
    if not env.applyCommand(AddLinkCommand(link, joint)):
        raise RuntimeError("could not attach the sphere")


def lvs_rebuild(locator: GeneralResourceLocator) -> dict[str, Any]:
    """checkTrajectory LVS_CONTINUOUS, and the same result built from its parts."""
    env = load(locator, IIWA_URDF_URL, IIWA_SRDF_URL)
    add_attached_sphere(env)
    manager = env.getContinuousContactManager()
    state_solver = env.getStateSolver()
    traj = np.linspace(JOINT_START, JOINT_END, NUM_STATES, dtype=np.float64)

    config = CollisionCheckConfig()
    config.type = CollisionEvaluatorType.LVS_CONTINUOUS
    config.check_program_mode = CollisionCheckProgramType.ALL
    config.exit_condition = CollisionCheckExitType.ALL
    trajectory_results, step_contacts = checkTrajectory(
        manager, state_solver, IIWA_JOINTS, traj, config
    )

    # --8<-- [start:lvs_rebuild]
    lvs = config.longest_valid_segment_length
    active_links = manager.getActiveCollisionObjects()
    # One map reused across steps, as utils.cpp:234 does: clear() keeps the keys
    # (with empty vectors), and == compares them
    step_results = ContactResultMap()
    steps_equal, rebuilt_pairs, rebuilt_counts = [], [], []
    for step in range(len(traj) - 1):
        step_results.clear()
        start, end = traj[step], traj[step + 1]
        # utils.cpp:236-243: sub-states every `lvs` of joint-space distance
        count = math.ceil(np.linalg.norm(end - start) / lvs) + 1
        substates = np.linspace(start, end, count)
        last_index = count - 1
        segment_dt = 1.0 / last_index

        for substep in range(last_index):
            state0 = state_solver.getState(IIWA_JOINTS, substates[substep]).link_transforms
            state1 = state_solver.getState(IIWA_JOINTS, substates[substep + 1]).link_transforms
            segment = checkTrajectorySegment(manager, state0, state1, config.contact_request)
            if not segment.empty():
                step_results.addInterpolatedCollisionResults(
                    segment, substep, last_index, active_links, segment_dt, False, None
                )
        steps_equal.append(step_results == step_contacts[step])
        # --8<-- [end:lvs_rebuild]
        rebuilt_pairs.append(step_results.size())
        rebuilt_counts.append(step_results.count())

    return {
        "steps_equal": steps_equal,
        "pairs_per_step": [contacts.size() for contacts in step_contacts],
        "rebuilt_pairs_per_step": rebuilt_pairs,
        "contact_count": trajectory_results.numContacts(),
        "rebuilt_contact_count": sum(rebuilt_counts),
        "contacts_per_step": [
            trajectory_results.steps[i].numContacts() for i in range(len(traj) - 1)
        ],
        "rebuilt_contacts_per_step": rebuilt_counts,
        "most_collisions_step": (
            trajectory_results.mostCollisionsStep().step,
            trajectory_results.mostCollisionsStep().numContacts(),
        ),
    }


def joint_trajectory() -> tuple[JointTrajectory, dict[str, Any]]:
    """EnvAddandRemoveTrajectoryLink's trajectory, built both ways."""
    # --8<-- [start:joint_trajectory]
    start = JointState(TRAJECTORY_JOINTS, np.zeros(2))
    end = JointState(TRAJECTORY_JOINTS, np.ones(2))

    pushed = JointTrajectory(TRAJECTORY_DESCRIPTION)
    pushed.push_back(start)
    pushed.push_back(end)
    trajectory = JointTrajectory([start, end], TRAJECTORY_DESCRIPTION)
    trajectory.uuid = pushed.uuid = str(uuid.uuid4())
    equal_by_constructor = trajectory == pushed

    positions = [state.position.tolist() for state in trajectory]
    midpoint = trajectory[1]
    midpoint.position = np.full(2, 0.5)
    getitem_is_copy = trajectory[1].position.tolist() == [1.0, 1.0]
    pushed[1] = midpoint
    after_setitem = pushed[1].position.tolist()
    pushed.clear()
    # --8<-- [end:joint_trajectory]
    return trajectory, {
        "equal_by_constructor": equal_by_constructor,
        "description": trajectory.description,
        "uuid_round_trip": trajectory.uuid == pushed.uuid,
        "len": len(trajectory),
        "positions": positions,
        "states_match_iteration": [s.position.tolist() for s in trajectory.states] == positions,
        "after_setitem": after_setitem,
        "getitem_is_copy": getitem_is_copy,
        "cleared": (len(pushed), pushed.empty()),
    }


def trajectory_link(locator: GeneralResourceLocator, trajectory: JointTrajectory) -> dict:
    """AddTrajectoryLinkCommand with each Method, replacing the same link."""
    env = load(locator, IIWA_URDF_URL, IIWA_SRDF_URL)
    scene_graph = env.getSceneGraph()
    active_links = sorted(
        {name for joint in TRAJECTORY_JOINTS for name in scene_graph.getJointChildrenIds(joint)}
    )
    with_collision = [name for name in active_links if scene_graph.getLink(name).collision]
    num_shapes = sum(len(scene_graph.getLink(name).collision) for name in with_collision)
    num_states = len(trajectory)
    expected = {
        "PER_STATE_OBJECTS": num_states * num_shapes,
        "PER_STATE_CONVEX_HULL": num_states,
        "GLOBAL_PER_LINK_CONVEX_HULL": len(with_collision),
        "GLOBAL_CONVEX_HULL": 1,
    }

    collisions = {}
    getters = {}
    for method in AddTrajectoryLinkCommand.Method:
        # --8<-- [start:trajectory_link]
        replace_allowed = method != AddTrajectoryLinkCommand.Method.PER_STATE_OBJECTS
        command = AddTrajectoryLinkCommand(
            TRAJECTORY_LINK, TRAJECTORY_PARENT, trajectory, replace_allowed, method
        )
        applied = env.applyCommand(command)
        collisions[method.name] = len(env.getLink(TRAJECTORY_LINK).collision)
        # --8<-- [end:trajectory_link]
        getters[method.name] = {
            "link_name": str(command.getLinkId()),
            "parent_link_name": str(command.getParentLinkId()),
            "trajectory_equal": command.getTrajectory() == trajectory,
            "replace_allowed": command.replaceAllowed(),
            "method": command.getMethod().name,
            "applied": applied,
        }
    return {
        "active_links_with_collision": with_collision,
        "collisions": collisions,
        "expected_collisions": expected,
        "getters": getters,
    }


def run() -> dict[str, Any]:
    """Run the example; return every result the test asserts on."""
    locator = GeneralResourceLocator(paths=[get_tesseract_resource_path()])
    trajectory, trajectory_result = joint_trajectory()
    return {
        "state": boxbot_checks(locator),
        "lvs": lvs_rebuild(locator),
        "joint_trajectory": trajectory_result,
        "trajectory_link": trajectory_link(locator, trajectory),
    }


def main() -> None:
    """Console entry point (`tesseract_trajectory_collision_example`): run() and print the results."""
    result = run()
    state = result["state"]
    for kind in ["discrete", "continuous", "fcl"]:
        print(
            f"checkTrajectoryState ({kind}): {state[kind]['pairs']}, "
            f"distance {state[kind]['distance']:.4f}; "
            f"boxbot_link disabled -> {state[kind]['pairs_with_boxbot_disabled']}"
        )
    print(f"Analytic penetration {-PENETRATION:.1f} m (FCL); Bullet's depth is backend-defined")
    print(f"checkTrajectorySegment: {state['segment_pairs']}")
    lvs = result["lvs"]
    print(
        f"checkTrajectory LVS_CONTINUOUS: pairs per step {lvs['pairs_per_step']}, "
        f"contacts per step {lvs['contacts_per_step']} ({lvs['contact_count']} total), "
        f"most in step {lvs['most_collisions_step'][0]}"
    )
    print(f"Rebuilt from checkTrajectorySegment, per step equal: {lvs['steps_equal']}")
    traj = result["joint_trajectory"]
    print(
        f"JointTrajectory '{traj['description']}': {traj['len']} states {traj['positions']}; "
        f"both constructors equal: {traj['equal_by_constructor']}"
    )
    links = result["trajectory_link"]
    print(f"Trajectory link from {links['active_links_with_collision']}:")
    for method, count in links["collisions"].items():
        print(
            f"  {method}: {count} collision shapes (expected {links['expected_collisions'][method]})"
        )


if __name__ == "__main__":
    main()
