#ifndef DRAW_BEHAVIOR_H_
#define DRAW_BEHAVIOR_H_

#include "task_tree/behavior.h"
#include <stdlib.h>
#include <time.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "gazebo_msgs/ModelStates.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "moveit_msgs/PickupAction.h"
#include "moveit_msgs/Grasp.h"
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

//x is positive to the robot's right and y is positive starting in front of the robot and coming toward it, z is positive going up
namespace task_net
{   
	
	class DrawBehavior: public Behavior
	{
	public:
		DrawBehavior(NodeId_t name, NodeList peers, NodeList children, NodeId_t parent,
				State_t state,
				std::string object,
				std::string to_draw,
				bool use_local_callback_queue = false,
				boost::posix_time::millisec mtime = boost::posix_time::millisec(50)) :
				draw_char(to_draw),
				Behavior(name, peers, children, parent,
					state,
					object,
					use_local_callback_queue,
					mtime) {srand(time(NULL));};

			virtual void Work();

	protected:

	private:

		std::string draw_char;

		//Space between each dot in the grid of 9 with which to draw the character
		const double width_between_dots = .03;
		const double height_between_dots = .035;

		//space between each of the separate characters
		const double width_to_next = .08;
		const double height_to_next = .08;

		//defining how far we can go before we have to wrap
		//NOTE: These values worked in sim so hopefully i will not have to continue to change them
		const double min_y = -.1;
		const double min_x = .5;

		//defining where the white_board starts. I think IRL I can just place the whiteboard in the right spot
		const double start_y = .5;
		const double start_x = .65;

		//this is whatever height that we measure we need
		const double z_height = 0.7;

		const double pick_up_height = .8;
	};
}
#endif // DRAW_BEHAVIOR_H_