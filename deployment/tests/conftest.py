"""Pytest configuration for Adelino deployment tests.

Adds the deployment directory to sys.path so that the modules
(which use relative imports like ``from config import ...``) can be
found by the test runner.
"""

import pathlib
import sys

_deployment_dir = str(pathlib.Path(__file__).resolve().parent.parent)
if _deployment_dir not in sys.path:
    sys.path.insert(0, _deployment_dir)
