import sys

# viser needs Python >= 3.11; on 3.9/3.10 the viewer is not installed (pyproject markers). Drop with #369.
collect_ignore_glob = ["test_*.py"] if sys.version_info < (3, 11) else []
