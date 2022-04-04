#include "behavior/grab_behavior.h"

namespace task_net {
	void GrabBehavior::Work()
	{
		ROS_WARN("Work started for: [%s]", this->name_->topic.c_str());

		//I think in general I'm going to assume the planning scene has already been loaded by a previous behavior

		moveit::planning_interface::MoveGroupInterface move_group_interface("left_arm");
		move_group_interface.setEndEffector("left_gripper");

		move_group_interface.setStartStateToCurrentState();
		
		geometry_msgs::Pose goal;


		tf2::Quaternion orientation;
		orientation.setRPY(0.0, M_PI / 2, 0.0);
		goal.orientation = tf2::toMsg(orientation);
		goal.position.x = .5;
		goal.position.y = -.1;
		goal.position.z = .7;

		move_group_interface.setPoseTarget(goal, "l_wrist_roll_link");
  	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

		//make a plan to move above the block
		for (int i = 0; i < 5; i++)
		{

			moveit::planning_interface::MoveItErrorCode ret_val = move_group_interface.plan(my_plan);

			if (ret_val == 0 || ret_val == 1)
			{
			  break; 
			}

			if (i == 4)
			{

			  ROS_FATAL("Plan could not be completed, returning in error.");

			  return;
			}
		}

		move_group_interface.execute(my_plan);

		ROS_WARN("Work Finished for: [%s]", this->name_->topic.c_str());
	}
}
