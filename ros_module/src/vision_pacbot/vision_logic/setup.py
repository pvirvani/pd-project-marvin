#!/usr/bin/env python3.8

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    scripts=['src/vision_logic/vision_logic.py'],
    packages=['vision_logic'],
    package_dir={'': 'src'},
    requires=['rospy', 'std_msgs']
)

setup(**setup_args)
