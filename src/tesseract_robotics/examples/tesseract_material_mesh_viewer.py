"""Mesh viewer example demonstrating GLB and DAE mesh loading with materials."""

import sys

from tesseract_robotics import get_tesseract_resource_path
from tesseract_robotics.tesseract_common import GeneralResourceLocator
from tesseract_robotics.tesseract_environment import Environment

shapes_urdf = """
<robot name="mesh_viewer" xmlns:tesseract="http://ros.org/wiki/tesseract" tesseract:make_convex="true">

  <link name="world"/>
  <link name="mesh_gltf2_link">
    <visual>
      <geometry>
        <mesh filename="package://tesseract/support/meshes/tesseract_material_mesh.glb"/>
      </geometry>
    </visual>
  </link>

  <joint name="mesh_gltf2_joint" type="revolute">
    <parent link="world"/>
    <child link="mesh_gltf2_link"/>
    <axis xyz="0 1 0"/>
    <origin xyz="0 -1 0.25"/>
    <limit effort="0" lower="-2.0944" upper="2.0944" velocity="6.2832"/>
  </joint>

  <link name="mesh_dae_link">
    <visual>
      <geometry>
        <mesh filename="package://tesseract/support/meshes/tesseract_material_mesh.dae"/>
      </geometry>
    </visual>
  </link>

  <joint name="mesh_dae_joint" type="revolute">
    <parent link="world"/>
    <child link="mesh_dae_link"/>
    <axis xyz="0 1 0"/>
    <origin xyz="0 1 0.25"/>
    <limit effort="0" lower="-2.0944" upper="2.0944" velocity="6.2832"/>
  </joint>

</robot>
"""


def run():
    t_env = Environment()

    # GeneralResourceLocator searches the support data given as `paths`
    locator = GeneralResourceLocator(paths=[get_tesseract_resource_path()])
    assert t_env.init(shapes_urdf, locator)
    return {"env": t_env, "locator": locator}


def main() -> int:
    results = run()

    from tesseract_robotics.viewer import TesseractViewer

    with TesseractViewer() as viewer:
        viewer.update_environment(results["env"])
        print(f"Viewer at {viewer.url}")
        input("Press Enter to exit...")
    return 0


if __name__ == "__main__":
    sys.exit(main())
