# nothing right now but like don't run this

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['htt_viz_py', 'rosgraph_msgs', 'qt_gui', 'python_qt_binding'],
    package_dir={'': 'src'}
)

setup(**d)