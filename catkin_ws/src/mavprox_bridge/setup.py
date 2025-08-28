#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['mavprox_bridge'],
    package_dir={'': 'viz'},
    scripts=[
        'viz/visualize_proximity.py',
        'viz/l1_tf_publisher.py',
    ],
)

setup(**setup_args)
