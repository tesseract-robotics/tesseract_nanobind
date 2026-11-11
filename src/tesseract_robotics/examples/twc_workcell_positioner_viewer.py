"""TWC Workcell viewer example.

Loads workcell URDFs from tesseract_ros_workcell and visualizes them.

Available workcells:
- positioner: ABB IRB4600 + IRBPA250 rotary positioner (2-DOF)
- rail: ABB IRB4600 on linear rail (1-DOF prismatic)

Usage:
    python twc_workcell_positioner_viewer.py [positioner|rail]
"""

import argparse
import math
import os
import re
import sys
from pathlib import Path
from typing import override

import numpy as np

from tesseract_robotics.common import ResourceLocator, SimpleLocatedResource
from tesseract_robotics.environment import Environment


def _find_twc_support() -> Path:
    """Find twc_support path from submodule or environment variable."""
    # Check env var first (allows override)
    if env_path := os.environ.get("TWC_SUPPORT_PATH"):
        return Path(env_path)

    # Default: the submodule, in the repo above this file (src/tesseract_robotics/examples/)
    for parent in Path(__file__).parents:
        submodule_path = parent / "ws/src/tesseract_ros_workcell/twc_support"
        if submodule_path.exists():
            return submodule_path

    # Fallback: common development location
    dev_path = Path.home() / "Code/CADCAM/tesseract_ros_workcell/twc_support"
    if dev_path.exists():
        return dev_path

    raise FileNotFoundError(
        "twc_support not found. Either:\n"
        "  1. Run 'git submodule update --init' to fetch the submodule\n"
        "  2. Set TWC_SUPPORT_PATH environment variable\n"
        "  3. Clone tesseract_ros_workcell to ~/Code/CADCAM/"
    )


TWC_SUPPORT_PATH = _find_twc_support()


class TwcResourceLocator(ResourceLocator):
    """Resource locator for twc_support package resources.

    Maps package://twc_support/... URLs to the actual filesystem path.
    Also handles URDF patching for Tesseract 0.33+ compatibility.
    """

    def __init__(self, twc_support_path: Path):
        super().__init__()
        self._twc_support_path = twc_support_path

    @override
    def locateResource(self, url: str):
        # Direct file path - return as-is
        if Path(url).exists():
            return SimpleLocatedResource(url, url, self)

        # Match package://twc_support/... pattern
        match = re.match(r"^package://twc_support/(.*)$", url)
        if match is None:
            print(f"TwcResourceLocator: unhandled URL pattern: {url}")
            return None

        # Resolve to filesystem path
        rel_path = match.group(1)
        full_path = self._twc_support_path / rel_path
        if not full_path.exists():
            print(f"TwcResourceLocator: file not found: {full_path}")
            return None

        return SimpleLocatedResource(url, str(full_path), self)

    def load_urdf(self, urdf_path: Path) -> str:
        """Load and patch URDF for Tesseract 0.33+ compatibility.

        Patches applied:
        - Adds tesseract:make_convex attribute if missing
        - Restores the rail cell's rail; see `restore_rail`
        - Removes empty <visual>/<collision> elements (invalid URDF)
        """
        content = urdf_path.read_text()

        # Add tesseract namespace and make_convex attribute
        if "tesseract:make_convex" not in content:
            content = re.sub(
                r'(<robot\s+name="[^"]*")',
                r'\1 xmlns:tesseract="http://ros.org/wiki/tesseract" tesseract:make_convex="true"',
                content,
                count=1,
            )

        content = restore_rail(content)

        # Remove empty visual/collision elements (upstream bug in some URDFs)
        # Matches: <visual>...<geometry>\s*</geometry>...</visual>
        content = re.sub(
            r"<visual>\s*<origin[^/]*/>\s*<geometry>\s*</geometry>\s*</visual>",
            "",
            content,
        )
        content = re.sub(
            r"<collision>\s*<origin[^/]*/>\s*<geometry>\s*</geometry>\s*</collision>",
            "",
            content,
        )

        return content


# The rail cell's URDF is older than its xacro: `rail_base_link` has empty geometry, so no rail
# draws, and the carriage's joint lacks the xacro's 0.2 m origin. The carriage spans -0.195 to
# 1.672 m of its own y, so that origin starts it on the rail, and a rail of length L takes it to
# L - 1.872 m. Only the 12 m mesh holds the URDF's 10 m joint limit.
RAIL_MESH = "package://twc_support/CAD/rail/gudel_tmf2/urdf/visual/base_link_12m.dae"
RAIL_JOINT_ORIGIN_XYZ = "0 0.2 0"


def restore_rail(urdf: str) -> str:
    """Give a rail cell's URDF the rail and joint origin its xacro describes.

    Leaves any other URDF, or one that has them already, as it is.
    """

    def fill_geometry(link: re.Match[str]) -> str:
        mesh = f'<geometry><mesh filename="{RAIL_MESH}"/></geometry>'
        return re.sub(r"<geometry>\s*</geometry>", mesh, link.group(0))

    urdf = re.sub(r'<link name="rail_base_link">.*?</link>', fill_geometry, urdf, flags=re.DOTALL)
    return re.sub(
        r'(<joint name="rail_joint_1" type="prismatic">\s*<origin rpy="0 0 0" xyz=)"0 0 0"',
        rf'\1"{RAIL_JOINT_ORIGIN_XYZ}"',
        urdf,
    )


WORKCELLS = {
    "positioner": {
        "urdf": "urdf/workcell_positioner.urdf",
        "srdf": "config/workcell_positioner.srdf",
        "joints": {
            "positioner_joint_1": (-math.pi, math.pi),
            "positioner_base_joint": (-math.pi / 2, math.pi / 2),
            "robot_joint_1": (-math.pi / 2, math.pi / 2),
            "robot_joint_2": (-math.pi / 4, math.pi / 4),
            "robot_joint_3": (-math.pi / 4, math.pi / 4),
            "robot_joint_4": (-math.pi / 2, math.pi / 2),
            "robot_joint_5": (-math.pi / 4, math.pi / 4),
            "robot_joint_6": (-math.pi, math.pi),
        },
    },
    "rail": {
        "urdf": "urdf/workcell_rail.urdf",
        "srdf": "config/workcell_rail.srdf",
        "joints": {
            "rail_joint_1": (0.0, 8.0),  # Linear rail; the joint allows 10 m
            "positioner_joint_1": (-math.pi, math.pi),  # Has positioner too
            "robot_joint_1": (-math.pi / 2, math.pi / 2),
            "robot_joint_2": (-math.pi / 4, math.pi / 4),
            "robot_joint_3": (-math.pi / 4, math.pi / 4),
            "robot_joint_4": (-math.pi / 2, math.pi / 2),
            "robot_joint_5": (-math.pi / 4, math.pi / 4),
            "robot_joint_6": (-math.pi, math.pi),
        },
    },
}


def animation_rows(workcell: str, duration: float = 60.0, fps: float = 30.0):
    """Sinusoidal joint motion, one row `[q..., t]` per frame.

    Returns:
        `(joint_names, rows)`.
    """
    animated_joints = WORKCELLS[workcell]["joints"]
    names = list(animated_joints)
    rows = []
    for t in np.arange(0.0, duration, 1.0 / fps):
        row = []
        for i, (lo, hi) in enumerate(animated_joints.values()):
            # Each joint oscillates at a different frequency for visual interest
            freq = 0.2 + i * 0.05
            mid = (lo + hi) / 2
            amp = (hi - lo) / 2
            row.append(mid + amp * math.sin(2 * math.pi * freq * t))
        rows.append([*row, t])
    return names, np.array(rows)


def run(workcell: str = "rail"):
    config = WORKCELLS[workcell]

    # --8<-- [start:load_workcell]
    urdf_path = TWC_SUPPORT_PATH / config["urdf"]
    srdf_path = TWC_SUPPORT_PATH / config["srdf"]

    print(f"Using twc_support from: {TWC_SUPPORT_PATH}")
    print(f"Workcell: {workcell}")
    print(f"Loading URDF: {urdf_path}")
    print(f"Loading SRDF: {srdf_path}")

    # Create resource locator (must keep reference alive!)
    locator = TwcResourceLocator(TWC_SUPPORT_PATH)

    # Load and patch URDF for Tesseract 0.33+ compatibility
    urdf_content = locator.load_urdf(urdf_path)

    # Initialize environment from URDF string
    t_env = Environment()
    if srdf_path.exists():
        srdf_content = srdf_path.read_text()
        success = t_env.init(urdf_content, srdf_content, locator)
    else:
        print("SRDF not found, loading URDF only")
        success = t_env.init(urdf_content, locator)
    assert success, f"failed to initialize the {workcell} workcell"
    # --8<-- [end:load_workcell]

    print(f"Environment initialized: {t_env.getName()}")
    print(f"Links: {len(list(t_env.getLinkIds()))}")
    print(f"Joints: {len(list(t_env.getJointIds()))}")
    return {"env": t_env, "locator": locator, "workcell": workcell}


def main() -> int:
    parser = argparse.ArgumentParser(description="TWC Workcell Viewer")
    parser.add_argument(
        "workcell",
        nargs="?",
        default="rail",
        choices=list(WORKCELLS.keys()),
        help="Workcell to load (default: rail)",
    )
    args = parser.parse_args()
    results = run(args.workcell)

    from tesseract_robotics.viewer import (
        TesseractViewer,
        Trajectory,
    )

    with TesseractViewer() as viewer:
        viewer.update_environment(results["env"])
        viewer.update_trajectory(Trajectory.from_rows(*animation_rows(args.workcell)))
        print(f"Viewer at {viewer.url}")
        input("Press Enter to exit...")
    return 0


if __name__ == "__main__":
    sys.exit(main())
