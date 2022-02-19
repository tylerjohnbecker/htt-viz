#!/usr/bin/python3

import os
import rospy
import rospkg
import sys

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from rqt_gui.main import Main

if __name__ == "__main__":
    # plugin = 'rqt_mypkg'
    plugin = 'qt_example'
    main = Main(filename=plugin)
    sys.exit(main.main(standalone=plugin))
    pass