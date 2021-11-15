cmake_minimum_required(VERSION 3.0.2)
project(htt_viz)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  genmsg
  actionlib_msgs
  roslib
  robotics_task_tree_msgs
)

find_package(Boost REQUIRED COMPONENTS 
  chrono
  thread
)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES htt_viz
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include_directories(include)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

##CPP executables
set(TaskNodes "src/task_tree/behavior.cc" "src/task_tree/node.cc")
#set(ActionNodes "src/behavior/action_behavior.cc" "src/behavior/move_behavior.cc")

add_executable(load src/htt_viz/loadTree.cpp ${TaskNodes})# ${ActionNodes})
target_link_libraries(load ${catkin_LIBRARIES} ${Boost_LIBRARIES} yaml-cpp)


#python executables
file (GLOB python_scripts_DIR
	"scripts/main.py"
)

catkin_python_setup()

catkin_install_python(PROGRAMS ${python_scripts_DIR}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)