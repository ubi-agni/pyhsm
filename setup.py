#!/usr/bin/env python

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    package_dir={"": "src"},
    packages=["hsm", "hsm.viewer", "hsm.viewer.xdot"],
    package_data={"hsm.viewer": ["gui/*"]},
)

setup(**d)
