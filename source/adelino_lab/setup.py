"""Installation script for the adelino_lab extension."""

import os

import toml
from setuptools import setup

EXTENSION_PATH = os.path.dirname(os.path.realpath(__file__))
EXTENSION_TOML_DATA = toml.load(os.path.join(EXTENSION_PATH, "config", "extension.toml"))

setup(
    name="adelino_lab",
    version=EXTENSION_TOML_DATA["package"]["version"],
    description=EXTENSION_TOML_DATA["package"]["description"],
    keywords=EXTENSION_TOML_DATA["package"]["keywords"],
    include_package_data=True,
    python_requires=">=3.10",
    # Note: isaaclab, isaaclab_assets, isaaclab_rl are installed separately
    # as editable packages from the Isaac Lab repo. Declaring them here would
    # cause pip to try to fetch them from PyPI which fails.
    install_requires=[
        "tensorboard",
    ],
    packages=["adelino_lab"],
    classifiers=[
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
    ],
    zip_safe=False,
)
