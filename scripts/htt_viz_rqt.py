#!/usr/bin/env python3

import sys

from htt_viz_rqt.my_module import MyPlugin
from rqt_gui.main import Main

plugin = 'htt_viz_rqt'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))