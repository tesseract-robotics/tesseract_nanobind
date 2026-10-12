"""
Environment State Example
=========================

Reads and sets an IIWA environment's state through every native `setState` /
`getState` / `getLinkTransforms` overload, with its first joint made FLOATING,
then queries joints, links, limits and timestamps, and looks up tool center
points (TCPs) with `findTCPOffset`.

C++ Reference:
    environment/test/tesseract_environment_unit.cpp  (getEnvironment :183-369, revision 3 :245-246,
        runGetLinkTransformsTest :125-174, tcpCallback :48-55, EnvFindTCPUnit :2745-2778)
    state_solver/test/state_solver_test_suite.h  (runSetFloatingJointStateTest :766-820)
    environment/src/environment.cpp  (currentStateChanged :651-654, findTCPOffset :832-862)
    common/test/tesseract_common_unit.cpp  (ManipulatorInfo :433-470)

Overview
--------
1. A `ReplaceJointCommand` makes `joint_a1` FLOATING at x = 1.25, as upstream's
   floating-joint test; the floating value then moves to y = 1.5 and z = 1.5,
   one `setState` overload per step.
2. The three `getState` overloads compute a state without changing the current
   one; the three `getLinkTransforms` overloads match `getState`.
3. Joint values by name, links moved by given joints, a `JointGroup` over given
   joints, joint limits, link visibility and collision flags, revisions and
   timestamps.
4. `findTCPOffset`: an explicit `Isometry3d`, the SRDF's `group_tcps`, then a
   registered callback; a link name or an unknown name raises `RuntimeError`.
5. `ManipulatorInfo`: the full constructor, `empty` and `getCombined`.

Key Concepts
------------
**Floating joints**:
    A FLOATING joint has no joint value; its pose is set as an `Isometry3d`
    through `floating_joints`. It is not an active joint, so it adds no
    Jacobian column and `getActiveJointNames()` leaves it out.

**Timestamps**:
    `setState` stamps both `getTimestamp()` and `getCurrentStateTimestamp()`
    from one `system_clock::now()` (environment.cpp:651-654), so they are equal
    right after it; a command stamps only `getTimestamp()`. The clock is not
    monotonic, so the example compares timestamps for equality only.

**TCP lookup order**:
    `findTCPOffset` returns an `Isometry3d` `tcp_offset` as is. A name must not
    be a link; it is looked up in the manipulator's `group_tcps`, then passed
    to each registered callback in turn. A callback that raises means "not
    found here".
"""

from pathlib import Path
from typing import Any

import numpy as np

from tesseract_robotics.tesseract_common import GeneralResourceLocator, Isometry3d, ManipulatorInfo
from tesseract_robotics.tesseract_environment import (
    ChangeLinkCollisionEnabledCommand,
    ChangeLinkVisibilityCommand,
    Environment,
    ReplaceJointCommand,
)
from tesseract_robotics.tesseract_scene_graph import Joint, JointType

URDF_URL = "package://tesseract/support/urdf/lbr_iiwa_14_r820.urdf"
SRDF_URL = "package://tesseract/support/urdf/lbr_iiwa_14_r820.srdf"
FLOATING_JOINT = "joint_a1"
# Upstream jvals (kinematics_test_utils.h:598-604) for the joints that stay
# active, joint_a2..joint_a7, rad
JOINT_VALUES = np.array([0.785398, -0.785398, 0.785398, -0.785398, 0.785398, -0.785398])
# Floating-joint origins, runSetFloatingJointStateTest (state_solver_test_suite.h:774, 797, 812), m
ORIGIN_X, ORIGIN_Y, ORIGIN_Z = 1.25, 1.5, 1.5
# runGetLinkTransformsTest isApprox precision (tesseract_environment_unit.cpp:155-158)
LINK_TRANSFORM_PRECISION = 1e-6
# EnvFindTCPUnit explicit / callback TCP offset (tesseract_environment_unit.cpp:52, 2752), m
TCP_Z = 0.1
# ManipulatorInfo unit test override offset (tesseract_common_unit.cpp:444), m
OVERRIDE_TCP_Z = 0.25
# Laser TCP rpy="0 1.57 0" (lbr_iiwa_14_r820.srdf:37), rad
LASER_PITCH = 1.57
# The SRDF parser builds the laser rotation from a half-angle quaternion
# (group_tool_center_points.cpp:135-141), numpy from cos/sin of the full angle. Each
# entry is at most 1 in magnitude; libm cos/sin are within 1 ulp, and the quaternion
# route adds at most 3 roundings (product, 1 - 2y², the sum), so 4 ulp of 1.0 bounds
# the difference. Measured on macOS arm64: 0.5 ulp.
TCP_ROTATION_TOL = 4 * np.finfo(np.float64).eps


def locate(locator: GeneralResourceLocator, url: str) -> Path:
    """File path of a `package://` resource; raises if the locator can't resolve it."""
    resource = locator.locateResource(url)
    if resource is None:
        raise FileNotFoundError(url)
    return Path(resource.getFilePath())


def translation(x: float = 0.0, y: float = 0.0, z: float = 0.0) -> Isometry3d:
    """Pure translation."""
    matrix = np.eye(4)
    matrix[:3, 3] = [x, y, z]
    return Isometry3d(matrix)


def is_approx(a: Isometry3d, b: Isometry3d) -> bool:
    """Eigen's `a.isApprox(b, LINK_TRANSFORM_PRECISION)` on the 4x4 matrices."""
    diff = np.linalg.norm(a.matrix - b.matrix)
    return bool(
        diff <= LINK_TRANSFORM_PRECISION * min(np.linalg.norm(a.matrix), np.linalg.norm(b.matrix))
    )


def transforms_agree(transforms: dict[str, Isometry3d], reference: dict[str, Isometry3d]) -> bool:
    """Same links, each transform within upstream's precision."""
    return transforms.keys() == reference.keys() and all(
        is_approx(tf, reference[link]) for link, tf in transforms.items()
    )


def find_tcp_error(env: Environment, manip_info: ManipulatorInfo) -> str:
    """The `RuntimeError` message `findTCPOffset` raises for `manip_info`."""
    try:
        env.findTCPOffset(manip_info)
    except RuntimeError as error:
        return str(error)
    raise AssertionError(f"findTCPOffset found {manip_info.tcp_offset!r}")


def laser_callback(manip_info: ManipulatorInfo) -> Isometry3d:
    """Upstream tcpCallback (tesseract_environment_unit.cpp:48-55)."""
    if manip_info.tcp_offset == "laser_callback":
        return translation(z=TCP_Z)
    raise RuntimeError("TCPCallback failed to find tcp!")


def run() -> dict[str, Any]:
    """Run the example; return every result the test asserts on."""
    locator = GeneralResourceLocator()
    env = Environment()
    env.init(locate(locator, URDF_URL), locate(locator, SRDF_URL), locator)
    revisions = {"after_init": (env.getRevision(), env.getInitRevision())}

    # --8<-- [start:floating_joint]
    joint = Joint(FLOATING_JOINT)
    joint.parent_to_joint_origin_transform = translation(ORIGIN_X)
    joint.parent_link_name = "base_link"
    joint.child_link_name = "link_1"
    joint.type = JointType.FLOATING
    env.applyCommand(ReplaceJointCommand(joint))

    names = env.getActiveJointNames()  # joint_a1 is no longer among them
    floating = env.getCurrentFloatingJointValues()
    floating_by_name = env.getCurrentFloatingJointValues([FLOATING_JOINT])
    # --8<-- [end:floating_joint]
    revisions["after_replace_joint"] = (env.getRevision(), env.getInitRevision())

    # --8<-- [start:set_state]
    origins = [
        translation(ORIGIN_X, ORIGIN_Y),
        translation(ORIGIN_X, ORIGIN_Y, ORIGIN_Z),
        translation(ORIGIN_X),
    ]
    joints = dict(zip(names, JOINT_VALUES))
    zeros = np.zeros(len(names))
    set_state = {
        "setState(floating)": lambda origin: env.setState({FLOATING_JOINT: origin}),
        "setState(dict, floating)": lambda origin: env.setState(joints, {FLOATING_JOINT: origin}),
        "setState(names, values, floating)": lambda origin: env.setState(
            names, JOINT_VALUES, {FLOATING_JOINT: origin}
        ),
    }
    steps = []
    previous = translation(ORIGIN_X)
    for (overload, set_with), origin in zip(set_state.items(), origins):
        # start from other joint values and the previous origin, so a no-op call is caught
        env.setState(names, zeros, {FLOATING_JOINT: previous})
        set_with(origin)
        # the floating-only overload leaves the joint values as they were
        expected = zeros if overload == "setState(floating)" else JOINT_VALUES
        steps.append(
            {
                "overload": overload,
                "origin": origin.translation.tolist(),
                "joints_read_back": np.array_equal(env.getCurrentJointValues(), expected),
                "floating_read_back": np.array_equal(
                    env.getCurrentFloatingJointValues()[FLOATING_JOINT].matrix, origin.matrix
                ),
                # a FLOATING joint has no motion: link_1 = base_link ∘ origin
                "link_1_at_origin": is_approx(
                    env.getLinkTransform("link_1"), env.getLinkTransform("base_link") * origin
                ),
            }
        )
        previous = origin
    # --8<-- [end:set_state]
    state_timestamps_equal = env.getCurrentStateTimestamp() == env.getTimestamp()

    # --8<-- [start:get_state]
    # The current state is JOINT_VALUES at origin x = 1.25. Each getState computes
    # another one, zero joint values and/or another origin, without setting it.
    other = {FLOATING_JOINT: translation(ORIGIN_X, ORIGIN_Y, ORIGIN_Z)}
    current = env.getState().link_transforms
    computed = {
        "getState(floating)": env.getState(other),  # the joints keep their current values
        "getState(dict, floating)": env.getState(dict(zip(names, zeros)), other),
        "getState(names, values, floating)": env.getState(names, zeros, other),
    }
    # --8<-- [end:get_state]
    base_link = current["base_link"]
    get_state: dict[str, bool] = {
        overload: is_approx(state.link_transforms["link_1"], base_link * other[FLOATING_JOINT])
        and np.array_equal(
            state.getJointValues(names),
            JOINT_VALUES if overload == "getState(floating)" else zeros,
        )
        for overload, state in computed.items()
    }
    get_state["current_state_unchanged"] = transforms_agree(env.getState().link_transforms, current)

    # --8<-- [start:link_transforms]
    link_transforms = {
        "getLinkTransforms()": dict(zip(env.getLinkNames(), env.getLinkTransforms())),
        "getLinkTransforms(names, values)": env.getLinkTransforms(names, zeros),
        "getLinkTransforms(names, values, floating)": env.getLinkTransforms(names, zeros, other),
    }
    # --8<-- [end:link_transforms]
    # (names, values) keeps the current origin; getState(names, values) does too
    references = {
        "getLinkTransforms()": current,
        "getLinkTransforms(names, values)": env.getState(names, zeros).link_transforms,
        "getLinkTransforms(names, values, floating)": computed[
            "getState(names, values, floating)"
        ].link_transforms,
    }
    link_transforms_agree = {
        overload: transforms_agree(transforms, references[overload])
        for overload, transforms in link_transforms.items()
    }

    # --8<-- [start:queries]
    current_joint_values = env.getCurrentJointValues(["joint_a3", "joint_a2"])
    active_links = env.getActiveLinkNames(["joint_a5"])  # the links joint_a5 moves
    static_links = env.getStaticLinkNames(["joint_a5"])
    group = env.getJointGroup("active_joints", names)
    limits = env.getJointLimits("joint_a2")

    flags_before = (env.getLinkVisibility("link_1"), env.getLinkCollisionEnabled("link_1"))
    env.applyCommands(
        [
            ChangeLinkVisibilityCommand("link_1", False),
            ChangeLinkCollisionEnabledCommand("link_1", False),
        ]
    )
    flags_after = (env.getLinkVisibility("link_1"), env.getLinkCollisionEnabled("link_1"))
    # --8<-- [end:queries]

    # --8<-- [start:find_tcp]
    explicit = env.findTCPOffset(
        ManipulatorInfo("manipulator", "unknown", "unknown", translation(z=TCP_Z))
    )
    laser = env.findTCPOffset(ManipulatorInfo("manipulator", "base_link", "tool0", "laser"))
    welder = env.findTCPOffset(ManipulatorInfo("manipulator", "base_link", "tool0", "welder"))

    by_callback = ManipulatorInfo("manipulator", "unknown", "unknown", "laser_callback")
    unresolved_before_callback = find_tcp_error(env, by_callback)
    env.addFindTCPOffsetCallback(laser_callback)
    callback = env.findTCPOffset(by_callback)
    # --8<-- [end:find_tcp]
    c, s = np.cos(LASER_PITCH), np.sin(LASER_PITCH)
    pitch = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
    tcp = {
        "explicit": explicit.translation.tolist(),
        "laser_translation": laser.translation.tolist(),
        "laser_rotation_error": float(np.abs(laser.linear - pitch).max()),
        "welder": welder.matrix.tolist(),
        "unresolved_before_callback": unresolved_before_callback,
        "callback": callback.translation.tolist(),
        "link_name": find_tcp_error(
            env, ManipulatorInfo("manipulator", "unknown", "unknown", "tool0")
        ),
        "unknown": find_tcp_error(
            env, ManipulatorInfo("manipulator", "unknown", "unknown", "unknown")
        ),
    }

    # --8<-- [start:manipulator_info]
    manip_info = ManipulatorInfo()
    default_empty = manip_info.empty()
    override = ManipulatorInfo("manipulator", "world", "tool0", translation(z=OVERRIDE_TCP_Z))
    override.manipulator_ik_solver = "OPWInvKin"
    override.working_frame = "base_link"
    combined = manip_info.getCombined(override)
    # --8<-- [end:manipulator_info]
    combined_offset = combined.tcp_offset
    if not isinstance(combined_offset, Isometry3d):
        raise TypeError(f"getCombined kept tcp_offset {combined_offset!r}, not the override's pose")
    manipulator_info = {
        "default_empty": default_empty,
        "combined": {
            "manipulator": combined.manipulator,
            "manipulator_ik_solver": combined.manipulator_ik_solver,
            "working_frame": combined.working_frame,
            "tcp_frame": combined.tcp_frame,
            "tcp_offset": combined_offset.translation.tolist(),
        },
        "combined_empty": combined.empty(),
        # tcp_frame, working_frame or manipulator missing
        "one_field_missing_empty": [
            ManipulatorInfo("manip", "world", "").empty(),
            ManipulatorInfo("manip", "", "tool0").empty(),
            ManipulatorInfo("", "world", "tool0").empty(),
        ],
    }

    return {
        "revisions": revisions,
        "active_joint_names": names,
        "floating_after_replace": {name: tf.translation.tolist() for name, tf in floating.items()},
        "floating_by_name_equal": floating_by_name.keys() == floating.keys()
        and np.array_equal(
            floating_by_name[FLOATING_JOINT].matrix, floating[FLOATING_JOINT].matrix
        ),
        "floating_steps": steps,
        "get_state": get_state,
        "link_transforms": link_transforms_agree,
        "current_joint_values": current_joint_values.tolist(),
        "link_names_by_joint": {"active": sorted(active_links), "static": sorted(static_links)},
        "joint_group": {
            "joint_names": group.getJointNames(),
            "tool0_matches_environment": is_approx(
                group.calcFwdKin(JOINT_VALUES)["tool0"], current["tool0"]
            ),
        },
        "joint_limits": {
            "lower": limits.lower,
            "upper": limits.upper,
            "velocity": limits.velocity,
            "acceleration": limits.acceleration,
            "jerk": limits.jerk,
        },
        "link_flags": {"before": flags_before, "after_commands": flags_after},
        "state_timestamps_equal": state_timestamps_equal,
        "tcp": tcp,
        "manipulator_info": manipulator_info,
    }


def main() -> None:
    """Console entry point (`tesseract_environment_state_example`): run() and print the results."""
    result = run()
    print(f"Revisions (revision, init revision): {result['revisions']}")
    print(
        f"Active joints {result['active_joint_names']}; floating {result['floating_after_replace']}"
    )
    for step in result["floating_steps"]:
        print(
            f"{step['overload']} -> joint_a1 origin {step['origin']}: joints read back "
            f"{step['joints_read_back']}, floating read back {step['floating_read_back']}, "
            f"link_1 = base_link ∘ origin {step['link_1_at_origin']}"
        )
    print(f"getState: {result['get_state']}")
    print(f"getLinkTransforms agrees with getState: {result['link_transforms']}")
    print(f"getCurrentJointValues([joint_a3, joint_a2]) = {result['current_joint_values']}")
    print(f"Links moved by joint_a5 / not: {result['link_names_by_joint']}")
    print(f"JointGroup over the active joints: {result['joint_group']}")
    print(f"joint_a2 limits: {result['joint_limits']}")
    print(f"link_1 (visible, collision enabled): {result['link_flags']}")
    print(
        f"After setState, getCurrentStateTimestamp() == getTimestamp(): {result['state_timestamps_equal']}"
    )
    tcp = result["tcp"]
    print(
        f"TCP explicit {tcp['explicit']}, laser {tcp['laser_translation']}, welder {tcp['welder']}"
    )
    print(f"  before the callback: RuntimeError {tcp['unresolved_before_callback']!r}")
    print(
        f"  callback {tcp['callback']}; link name: {tcp['link_name']!r}; unknown: {tcp['unknown']!r}"
    )
    print(f"ManipulatorInfo: {result['manipulator_info']}")


if __name__ == "__main__":
    main()
