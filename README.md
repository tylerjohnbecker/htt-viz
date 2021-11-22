# Basic run instructions and instructions on where to put stuff

For everything that just needs to be run pop it in the main file.

If you want to work away from the main file the other files we can use need to be put in /src/htt_viz_py
because of how ROS packages work. And then if you want to include them follow this example.

filesystem:
-src
-->htt_viz_py
-->-->NodeView.py
-scripts
-->Main.py

and in Main.py you would include NodeView like so:
import htt_viz_py.NodeView
or
from htt_viz_py.NodeView import NodeView

#Main running instruction for the package
on the command line with roscore running
rosrun htt_viz Main.py

Do this instead of the normal python instructions so that we can test with rosnodes running when we add
that functionality later.