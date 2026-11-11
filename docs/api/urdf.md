# tesseract_robotics.urdf

<!-- snippet: ambient
from pathlib import Path

from tests.docs.doc_assets import enter_scratch_dir, write_stl

enter_scratch_dir()
write_stl(Path("model.stl"))
Path("out_package").mkdir()
-->

URDF parsing and writing: `parseURDFString` / `parseURDFFile` build a `SceneGraph`,
`writeURDFFile` writes one back to `<package_path>/urdf/<urdf_name>.urdf` and returns that path
(or raises `URDFWriteError` naming the cause), and `writeMeshToFile` writes a mesh as the PLY file a written
URDF references.

```python
from tesseract_robotics import get_tesseract_resource_path
from tesseract_robotics.common import GeneralResourceLocator
from tesseract_robotics.geometry import Box
from tesseract_robotics.scene_graph import Collision, Joint, JointType, Link, SceneGraph, Visual
from tesseract_robotics.urdf import parseURDFFile, writeURDFFile

locator = GeneralResourceLocator(paths=[get_tesseract_resource_path()])  # the [support] extra
urdf = locator.locateResource("package://tesseract/support/urdf/lbr_iiwa_14_r820.urdf").getFilePath()

scene_graph = parseURDFFile(urdf, locator)
print(scene_graph.getRoot())

# Write a graph with primitive geometry; the package directory must exist
graph = SceneGraph()
graph.setName("example")
base = Link("base_link")
visual, collision = Visual(), Collision()
visual.geometry = collision.geometry = Box(0.1, 0.1, 0.1)
base.addVisual(visual)
base.addCollision(collision)
graph.addLink(base)
graph.addLink(Link("tip"))
joint = Joint("base_to_tip")
joint.type = JointType.FIXED
joint.parent_link_id = "base_link"
joint.child_link_id = "tip"
graph.addJoint(joint)
path = writeURDFFile(graph, "out_package", "robot")  # urdf_name is a stem
assert path.name == "robot.urdf"
```

Since tesseract 0.33 the `<robot>` element needs `tesseract:make_convex="true"` or `"false"`
(`xmlns:tesseract="http://ros.org/wiki/tesseract"`), and a URDF with more than one link needs
joints connecting them. Mesh URLs resolve through the locator. Prefer `package://` URLs:
upstream's `file://` handling expects `file:///` followed by an absolute POSIX path
(resource_locator.cpp:161–163), so `file://C:\...` does not resolve on Windows.

## writeMeshToFile

```python
from tesseract_robotics.geometry import createMeshFromPath
from tesseract_robotics.urdf import writeMeshToFile

(mesh,) = createMeshFromPath("model.stl")  # your own mesh file
writeMeshToFile(mesh, "model.ply")
```

The file is always an ASCII PLY, whatever the extension of `filepath` (urdf/src/utils.cpp:140–155).
`filepath` is a `str`, the native type. `None` for `mesh` raises `TypeError`; a path that cannot be
written (a missing directory, say) raises `RuntimeError("Could not export file")`.

The string helpers in the same header (`toString`, `trailingSlash`, `noTrailingSlash`,
`noLeadingSlash`, `makeURDFFilePath`) are not bound: Python's string formatting and `pathlib`
cover them.

## Reference

::: tesseract_robotics.urdf
