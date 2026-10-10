# tesseract_robotics.tesseract_urdf

URDF parsing and writing: `parseURDFString` / `parseURDFFile` build a `SceneGraph`,
`writeURDFFile` writes one back, and `writeMeshToFile` writes a mesh as the PLY file a written
URDF references.

```python
from tesseract_robotics.tesseract_common import GeneralResourceLocator
from tesseract_robotics.tesseract_urdf import parseURDFFile, writeURDFFile

locator = GeneralResourceLocator()
urdf = locator.locateResource("package://tesseract/support/urdf/lbr_iiwa_14_r820.urdf").getFilePath()

scene_graph = parseURDFFile(urdf, locator)
writeURDFFile(scene_graph, "/path/to/package", "robot.urdf")
```

Since tesseract 0.33 the `<robot>` element needs `tesseract:make_convex="true"` or `"false"`
(`xmlns:tesseract="http://ros.org/wiki/tesseract"`), and a URDF with more than one link needs
joints connecting them. Mesh URLs resolve through the locator. Prefer `package://` URLs:
upstream's `file://` handling expects `file:///` followed by an absolute POSIX path
(resource_locator.cpp:161–163), so `file://C:\...` does not resolve on Windows.

## writeMeshToFile

```python
from tesseract_robotics.tesseract_geometry import createMeshFromPath
from tesseract_robotics.tesseract_urdf import writeMeshToFile

(mesh,) = createMeshFromPath("model.stl")
writeMeshToFile(mesh, "model.ply")
```

The file is always an ASCII PLY, whatever the extension of `filepath` (urdf/src/utils.cpp:140–155).
`filepath` is a `str`, the native type. `None` for `mesh` raises `TypeError`; a path that cannot be
written (a missing directory, say) raises `RuntimeError("Could not export file")`.

The string helpers in the same header (`toString`, `trailingSlash`, `noTrailingSlash`,
`noLeadingSlash`, `makeURDFFilePath`) are not bound: Python's string formatting and `pathlib`
cover them.

## Auto-generated API Reference

::: tesseract_robotics.tesseract_urdf._tesseract_urdf
    options:
      show_root_heading: false
      show_source: false
      members_order: source
