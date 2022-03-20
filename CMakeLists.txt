cmake_minimum_required(VERSION 3.0.2)
project(htt_viz)

set(CMAKE_CXX_STANDARD 20)
#set(yaml-cpp_DIR "C:/opt/yaml-cpp-yaml-cpp-0.6.0/build")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  genmsg
  message_generation
  actionlib_msgs
  std_msgs
  geometry_msgs
  rosgraph_msgs
  roslib
)

find_package(Boost REQUIRED COMPONENTS 
  chrono
  thread
)

catkin_python_setup()

add_message_files(
  FILES
  ActuatorRequest.msg
  ActuatorState.msg
  Actuator.msg
  State.msg
  NodeBitmask.msg
  ControlMessage.msg
  ObjStatus.msg
  hold_status.msg
  Object.msg
  Robot.msg
  SimState.msg
  PeerSimState.msg
  Issue.msg
  Resolution.msg
  Human_Resolution.msg
)

add_service_files(FILES
	Update.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  LIBRARIES htt_viz
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

add_definitions(-DBOOST_LOG_DYN_LINK=1)

include_directories(include)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

##CPP executables
set(TaskNodes "src/task_tree/behavior.cc" "src/task_tree/node.cc")
set(ActionNodes "src/behavior/dummy_behavior.cc" "src/behavior/move_behavior.cc" "src/behavior/grab_behavior.cc")

add_executable(load src/htt_viz/loadTree.cpp ${TaskNodes} ${ActionNodes})
target_link_libraries(load ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(load ${catkin_EXPORTED_TARGETS} htt_viz_generate_messages_cpp)


#python executables
file (GLOB python_scripts_DIR
	"scripts/main.py"
  "scripts/htt_viz_rqt.py"
)

catkin_install_python(PROGRAMS ${python_scripts_DIR}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# TODO: Get this to work
#
# See `http://wiki.ros.org/rqt/Tutorials/Create%20your%20new%20rqt%20plugin` for help
#
# install(PROGRAMS scripts/htt_viz_rqt
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# Copy RQT resources
install(DIRECTORY
   resource
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES
   plugin.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
