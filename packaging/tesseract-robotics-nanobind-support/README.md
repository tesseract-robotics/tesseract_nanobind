# tesseract-robotics-nanobind-support

The robot models and meshes of [tesseract](https://github.com/tesseract-robotics/tesseract)'s
`support` package (URDFs, SRDFs, plugin configs, meshes), packaged for
[tesseract-robotics-nanobind](https://pypi.org/project/tesseract-robotics-nanobind/), whose
examples and tests load robots from it through `package://tesseract/support/...`.

```bash
pip install tesseract-robotics-nanobind[support]
```

`import tesseract_robotics` then points `TESSERACT_SUPPORT_DIR` and `TESSERACT_RESOURCE_PATH` at
this package, unless they are already set.
