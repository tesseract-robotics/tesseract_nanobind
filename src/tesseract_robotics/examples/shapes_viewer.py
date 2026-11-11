"""Simple shapes viewer example using tesseract_robotics with nanobind bindings."""

import sys

from tesseract_robotics import get_tesseract_resource_path
from tesseract_robotics.common import GeneralResourceLocator
from tesseract_robotics.environment import Environment
from tesseract_robotics.planning.errors import EnvironmentInitError

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


def run():
    # --8<-- [start:setup]
    t_env = Environment()

    # locator must be kept alive by maintaining a reference
    locator = GeneralResourceLocator(paths=[get_tesseract_resource_path()])
    if not t_env.init(shapes_urdf, locator):
        raise EnvironmentInitError("failed to initialize the shapes environment")
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
