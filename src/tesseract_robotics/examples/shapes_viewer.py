"""Simple shapes viewer example using tesseract_robotics with nanobind bindings."""

import os
import re
import sys
import traceback
from typing import override

from tesseract_robotics import get_tesseract_support_path
from tesseract_robotics.common import ResourceLocator, SimpleLocatedResource
from tesseract_robotics.environment import Environment

shapes_urdf = """
<robot name="multipleshapes" xmlns:tesseract="http://ros.org/wiki/tesseract" tesseract:make_convex="true">

  <link name="world"/>
  <link name="cylinder_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
  </link>

  <joint name="clyinder_joint" type="revolute">
    <parent link="world"/>
    <child link="cylinder_link"/>
    <axis xyz="0 1 0"/>
    <limit effort="0" lower="-2.0944" upper="2.0944" velocity="6.2832"/>
  </joint>

  <link name="box_link">
    <visual>
      <geometry>
        <box size="0.6 0.1 0.2"/>
      </geometry>
      <material name="green">
        <color rgba="0 0.8 0 1"/>
      </material>
    </visual>
  </link>

  <joint name="box_joint" type="revolute">
    <parent link="world"/>
    <child link="box_link"/>
    <origin xyz="1 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="0" lower="-2.0944" upper="2.0944" velocity="6.2832"/>
  </joint>

  <link name="sphere_link">
    <visual>
      <geometry>
        <sphere radius="0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
  </link>

  <joint name="sphere_joint" type="revolute">
    <parent link="world"/>
    <child link="sphere_link"/>
    <origin xyz="2 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="0" lower="-2.0944" upper="2.0944" velocity="6.2832"/>
  </joint>

</robot>
"""


class TesseractSupportResourceLocator(ResourceLocator):
    """Resource locator for tesseract_support package resources."""

    def __init__(self):
        super().__init__()

    @override
    def locateResource(self, url):
        try:
            if os.path.exists(url):
                return SimpleLocatedResource(url, url, self)

            url_match = re.match(r"^package:\/\/tesseract_support\/(.*)$", url)
            if url_match is None:
                print(f"url_match failed for: {url}")
                return None

            tesseract_support = str(get_tesseract_support_path())

            filename = os.path.join(tesseract_support, os.path.normpath(url_match.group(1)))
            return SimpleLocatedResource(url, filename, self)
        except Exception:
            traceback.print_exc()
            return None


def run():
    # --8<-- [start:setup]
    t_env = Environment()

    # locator must be kept alive by maintaining a reference
    locator = TesseractSupportResourceLocator()
    assert t_env.init(shapes_urdf, locator)
    # --8<-- [end:setup]
    return {"env": t_env, "locator": locator}


def main() -> int:
    results = run()

    from tesseract_robotics.viewer import TesseractViewer

    # --8<-- [start:serve]
    with TesseractViewer() as viewer:
        viewer.update_environment(results["env"])
        print(f"Viewer at {viewer.url}")
        input("Press Enter to exit...")
    # --8<-- [end:serve]
    return 0


if __name__ == "__main__":
    sys.exit(main())
