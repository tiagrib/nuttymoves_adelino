"""Script to list all registered Adelino gymnasium environments.

Must be run through Isaac Sim runtime (pxr module required at import time).
"""

"""Launch Isaac Sim Simulator first."""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="List registered Adelino environments.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# launch omniverse app (headless)
args_cli.headless = True
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import gymnasium as gym

# Import adelino_lab to trigger gym.register() calls
import adelino_lab  # noqa: F401

SEARCH_PREFIX = "Adelino-"

print(f"\nRegistered environments matching '{SEARCH_PREFIX}':")
print("-" * 50)
for env_id in sorted(gym.registry.keys()):
    if env_id.startswith(SEARCH_PREFIX):
        print(f"  {env_id}")

simulation_app.close()
