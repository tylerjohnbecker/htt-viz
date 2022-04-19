/**
* Description: This behaviour commands the robot to grab
* 
* Created By: Philip
* Created On: 12/06/21
* Last Updated: 12/06/21
*/

#ifndef GRAB_BEHAVIOR_H_
#define GRAB_BEHAVIOR_H_

#include "task_tree/behavior.h"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/Pose.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "moveit_msgs/PickupAction.h"
#include "moveit_msgs/Grasp.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit/kinematic_constraints/utils.h>
#include "moveit_msgs/MoveGroupAction.h"
#include "moveit_msgs/OrientationConstraint.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pluginlib/class_loader.h>
#include "pr2_common_action_msgs/TuckArmsAction.h"
#include "pr2_controllers_msgs/Pr2GripperCommandAction.h"
#include "actionlib/client/simple_action_client.h"
#include "moveit_msgs/OrientationConstraint.h"


namespace task_net {

	class GrabBehavior : public Node {
	public:
		GrabBehavior(NodeId_t name, NodeList peers, NodeList children, NodeId_t parent,
			State_t state,
			std::string object,
  			WorkMutex* wm,
			bool use_local_callback_queue = false,
			boost::posix_time::millisec mtime = boost::posix_time::millisec(50)) :
			Node(name, peers, children, parent,
				state,
				object,
				wm,
				use_local_callback_queue,
				mtime) {};

		virtual void Work();
	protected:

	private:
	};
}

#endif //GRAB_BEHAVIOR_H_
