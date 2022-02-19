#!/usr/bin/env python3

# Setup file for building this package.
# You shouldn't run this manually.
# See `http://docs.ros.org/en/jade/api/catkin/html/user_guide/setup_dot_py.html` for more info.

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=[
		'htt_viz_py', 
		
		# Nathaniel: Hope this wan't important, it broke my build
		# 'rosgraph_msgs', 
		# 'qt_gui', 
		# 'python_qt_binding', 
		
		'htt_viz_rqt',
	],
    package_dir={'': 'src'}
)

setup(**d)