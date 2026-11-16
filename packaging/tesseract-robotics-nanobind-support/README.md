# tesseract-robotics-nanobind-support

The robot models and meshes of [tesseract](https://github.com/tesseract-robotics/tesseract)'s
`support` package (URDFs, SRDFs, plugin configs, meshes), packaged for
[tesseract-robotics-nanobind](https://pypi.org/project/tesseract-robotics-nanobind/), whose
examples and tests load robots from it through `package://tesseract/support/...`.

`pip install tesseract-robotics-nanobind` installs it: the bindings depend on it at their own
version. `tesseract_robotics.get_tesseract_resource_path()` returns its directory, for a
`GeneralResourceLocator(paths=[...])`.
