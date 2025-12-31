# Installation

## Requirements

- Python 3.9+ (3.12 recommended)
- CMake 3.16+
- C++17 compiler

## Install from PyPI

```bash
pip install tesseract-robotics
```

## Install from Source

### 1. Clone the Repository

```bash
git clone https://github.com/tesseract-robotics/tesseract_nanobind.git
cd tesseract_nanobind
```

### 2. Create Conda Environment

```bash
conda env create -f environment.yml
conda activate tesseract_nb
```

### 3. Build C++ Dependencies

```bash
./scripts/build_tesseract_cpp.sh
```

This builds the Tesseract C++ libraries in `ws/install/`.

### 4. Install Python Package

```bash
pip install -e .
```

## Verify Installation

```python
import tesseract_robotics
print(tesseract_robotics.__version__)

# Test with a bundled robot
from tesseract_robotics.planning import Robot
robot = Robot.from_tesseract_support("abb_irb2400")
print(f"Loaded robot with {len(robot.get_joint_names('manipulator'))} joints")
```

## Environment Variables

The package auto-configures these at import time:

| Variable | Purpose |
|----------|---------|
| `TESSERACT_SUPPORT_DIR` | Robot URDF/mesh resources |
| `TESSERACT_TASK_COMPOSER_CONFIG_FILE` | Task composer plugins |
| `TESSERACT_KINEMATICS_PLUGIN_PATH` | Kinematics plugin libraries |
| `TESSERACT_CONTACT_MANAGERS_PLUGIN_PATH` | Collision plugin libraries |

For development, you can also source `env.sh`:

```bash
source env.sh
python examples/freespace_ompl_example.py
```

## Troubleshooting

### Plugin Loading Errors

If you see errors about missing plugins:

```
Error: Failed to load plugin library
```

Ensure environment variables are set. The package should auto-configure these, but for editable installs you may need:

```bash
source env.sh
```

### OpenMP Crashes (macOS)

If you get crashes related to OpenMP on macOS, ensure you're using conda's `llvm-openmp`:

```bash
conda install llvm-openmp
```

The build scripts are configured to use `$CONDA_PREFIX/lib/libomp.dylib`.

### RTTI/typeinfo Errors

If TaskComposer fails with "Input is not a Composite Instruction", rebuild with visibility settings:

```bash
# In build_tesseract_cpp.sh and CMakeLists.txt
-DCMAKE_CXX_VISIBILITY_PRESET=default
```
