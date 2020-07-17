#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

CONFIG = generate_distutils_setup(
    packages=['costmap_based_vel_filter'],
    package_dir={'': 'src'}
)

setup(**CONFIG)
