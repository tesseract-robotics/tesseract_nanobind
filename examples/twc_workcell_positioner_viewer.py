"""TWC Workcell Positioner viewer example.

Loads the workcell_positioner URDF from tesseract_ros_workcell and visualizes it.
This workcell includes:
- ABB IRB4600-60/205 robot (6-DOF) on pedestal
- ABB IRBPA250 D1000 rotary positioner (2-DOF)
- ATI tool changers, PushCorp EOAT, safety fence
"""

import math
import os
import re
import sys
import time
from pathlib import Path

import numpy as np

from tesseract_robotics.tesseract_common import ResourceLocator, SimpleLocatedResource
from tesseract_robotics.tesseract_environment import Environment

# Viewer (skip import in pytest)
TesseractViewer = None
if "pytest" not in sys.modules:
    from tesseract_robotics_viewer import TesseractViewer


def _find_twc_support() -> Path:
    """Find twc_support path from submodule or environment variable."""
    # Check env var first (allows override)
    if env_path := os.environ.get("TWC_SUPPORT_PATH"):
        return Path(env_path)

    # Default: submodule location relative to this file
    submodule_path = Path(__file__).parent.parent / "ws/src/tesseract_ros_workcell/twc_support"
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

        Adds tesseract:make_convex attribute if missing.
        """
        content = urdf_path.read_text()

        # Already patched
        if "tesseract:make_convex" in content:
            return content

        # Add tesseract namespace and make_convex attribute to robot element
        return re.sub(
            r'(<robot\s+name="[^"]*")',
            r'\1 xmlns:tesseract="http://ros.org/wiki/tesseract" tesseract:make_convex="true"',
            content,
            count=1,
        )


def animate_joints(viewer, duration: float = 30.0, fps: float = 30.0):
    """Animate robot and positioner joints with sinusoidal motion."""
    # Define joints to animate with conservative limits (subset for smooth demo)
    # Positioner: ±2π rad, Robot: varies by joint
    animated_joints = {
        "positioner_joint_1": (-math.pi, math.pi),
        "positioner_joint_2": (-math.pi / 2, math.pi / 2),
        "robot_joint_1": (-math.pi / 2, math.pi / 2),
        "robot_joint_2": (-math.pi / 4, math.pi / 4),
        "robot_joint_3": (-math.pi / 4, math.pi / 4),
        "robot_joint_4": (-math.pi / 2, math.pi / 2),
        "robot_joint_5": (-math.pi / 4, math.pi / 4),
        "robot_joint_6": (-math.pi, math.pi),
    }

    # Animation loop
    dt = 1.0 / fps
    start_time = time.time()

    print(f"Animating joints for {duration}s... Press Ctrl+C to stop.")
    try:
        while time.time() - start_time < duration:
            t = time.time() - start_time

            # Compute joint positions using sinusoidal motion at different frequencies
            names = []
            positions = []
            for i, (name, (lo, hi)) in enumerate(animated_joints.items()):
                # Each joint oscillates at a different frequency for visual interest
                freq = 0.2 + i * 0.05
                mid = (lo + hi) / 2
                amp = (hi - lo) / 2
                pos = mid + amp * math.sin(2 * math.pi * freq * t)
                names.append(name)
                positions.append(pos)

            viewer.update_joint_positions(names, np.array(positions))
            time.sleep(dt)
    except KeyboardInterrupt:
        print("\nAnimation stopped.")


def main():
    headless = "pytest" in sys.modules

    urdf_path = TWC_SUPPORT_PATH / "urdf/workcell_positioner.urdf"
    srdf_path = TWC_SUPPORT_PATH / "config/workcell_positioner.srdf"

    print(f"Using twc_support from: {TWC_SUPPORT_PATH}")
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
        success = t_env.initFromUrdfSrdf(urdf_content, srdf_content, locator)
    else:
        print("SRDF not found, loading URDF only")
        success = t_env.initFromUrdf(urdf_content, locator)

    if not success:
        print("ERROR: Failed to initialize environment")
        sys.exit(1)

    print(f"Environment initialized: {t_env.getName()}")
    print(f"Links: {len(list(t_env.getLinkNames()))}")
    print(f"Joints: {len(list(t_env.getJointNames()))}")

    if not headless:
        print("\nStarting viewer at http://127.0.0.1:8000 ...")
        viewer = TesseractViewer()
        viewer.update_environment(t_env, [0, 0, 0])
        viewer.start_serve_background()

        # Animate joints
        animate_joints(viewer, duration=60.0)

        print("\nAnimation complete. Press Enter to exit...")
        input()
        viewer.close()
    else:
        print("twc_workcell_positioner_viewer.py: PASSED (headless)")


if __name__ == "__main__":
    main()
