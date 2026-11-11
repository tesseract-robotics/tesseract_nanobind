# trajopt_ifopt Python bindings - variables, constraints, and costs for IFOPT-based optimization.
# A package, not a bare extension: python_constraints.py lives beside the compiled module, and an
# extension named trajopt_ifopt would shadow this directory (the import would fail).

from ._trajopt_ifopt import *  # noqa: F403
