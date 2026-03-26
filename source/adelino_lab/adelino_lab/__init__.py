"""Adelino Lab: Isaac Lab environments for the Adelino 5-DOF walking manipulator."""

import os

import toml

# Read extension metadata
EXT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../"))
EXT_METADATA = toml.load(os.path.join(EXT_DIR, "config", "extension.toml"))
__version__ = EXT_METADATA["package"]["version"]

# Auto-import all task subpackages to trigger gym.register() calls
from isaaclab_tasks.utils import import_packages

import_packages(__name__, blacklist_pkgs=["mdp", "agents"])
