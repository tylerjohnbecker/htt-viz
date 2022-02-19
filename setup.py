#!/usr/bin/env python3

# Setup file for building this package.
# You shouldn't run this manually.
# See `http://docs.ros.org/en/jade/api/catkin/html/user_guide/setup_dot_py.html` for more info.

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# In the future, use `setup_tools` to install python deps
# See `https://docs.ros.org/en/api/catkin/html/howto/format2/installing_python.html`.

d = generate_distutils_setup(
    packages=[
		'htt_viz_py', 
		'htt_viz_rqt',
		
		# Nathaniel: Hope this wan't important, it broke my build
		# 'rosgraph_msgs', 
		# 'qt_gui', 
		# 'python_qt_binding', 
	],
    package_dir={'': 'src'}
)

setup(**d)