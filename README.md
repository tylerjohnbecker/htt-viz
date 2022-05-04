# Description
Hierarchical Task Tree Visualizer (HTT-Viz) is a visual editing software built for creating, editing, and debugging Hierarchical Task Trees (HTT's). HTT's serve as a control mechanism in robotics which enables the programmer to quickly and efficintly enforce ordering constraints on small reusable actions to create larger and more complex tasks. HTT-Viz trivializees the incorporation of HTT's into any project by adding a visual component to the creatoin and maintenance of the trees. The intended usage of HTT-Viz is to simplify the use of HTT's through the utilization of a visual interface. This interface allows for simplify editing of a tree structure, as opposed to manually writing the trees in a text document.

# Dependencies
HTT Architecture:
- Boost
- MoveIt
- PR2 libs

HTT-Viz:
- pyyaml
- py-qt5
- graphviz

# Installation
This is a rospkg made for ROS Noetic. It needs to be run along with roscore.
Clone this repository from main into the src folder of the catkin_ws. Run catkin_make.

# Usage
HTT Architecture: 
tree.launch launches the HTT with the given file name of the chosen tree. Also launches the parameter server which is used for the draw_behavior. 
whiteboard_world.launch launches gazebo with the world used for testing the draw_behavior.
move_base.launch launches the moveit server which is used in the draw_behavior.

The general order of running for sim is to start the world, then the moveit server, and then the HTT.

The architecture can run trees made by the user if they place them in the trees folder and call their name when launching it.

HTT-Viz:
After running catkin_make the software can be run with the following: 
- rosrun htt_viz htt_viz

In order to display the visual feedback of a running tree, the tree that is running on the architecture needs to be loaded into HTT-Viz.
