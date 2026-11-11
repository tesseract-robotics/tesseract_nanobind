"""trajopt_ifopt Python bindings: variables, constraints and costs for TrajOpt's SQP.

Rules written in Python live in `tesseract_robotics.trajopt_ifopt.python_constraints`.
"""

# A package, not a bare extension: python_constraints.py lives beside the compiled module, and an
# extension named trajopt_ifopt would shadow this directory (the import would fail).
from tesseract_robotics.trajopt_ifopt._trajopt_ifopt import *
