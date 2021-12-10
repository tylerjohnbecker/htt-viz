#HTT_VIZ

A graphical user interface which allows the user to edit and save Hierarchical Task Trees before deploying them to the Robot.

#Dependencies

This project depends on BOOST for the cpp files.
- https://www.boost.org/doc/libs/1_62_0/more/getting_started/windows.html#get-boost

After that is install simply build it using catkin.

# Basic instructions for adding files

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

for the HTT itself it can be run with the following command for the tree.launch file:
roslaunch htt_viz tree.launch test_num:=k
where k is the last digit of the file with the naming convention "test_" + k + ".yaml"

Note: for now we can't save the files so we can enforce this convention, however, I'll change this to an actual file_name once we have the save functionality 

Do this instead of the normal python instructions so that we can test with rosnodes running when we add that functionality later.
