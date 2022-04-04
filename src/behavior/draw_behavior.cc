#include "behavior/draw_behavior.h"

namespace task_net {
	void DrawBehavior::Work()
	{
		ROS_WARN("Work started for: [%s]", this->name_->topic.c_str());

		double x_offset, y_offset;
		this->local_.getParam("x_offset", x_offset);
		this->local_.getParam("y_offset", y_offset);

		if (start_x + x_offset <= this->min_x)
		{
			ROS_INFO("There is no space to draw so please erase the board...");
			//std::cin;

			x_offset = 0.0;
			y_offset = 0.0;
			ROS_INFO("Starting again from [x, y]: [%f, %f]", x_offset, y_offset); 
		}

		//ROS_INFO("Drawing at [x, y]: [%f, %f]", x_offset, y_offset);

		moveit::planning_interface::MoveGroupInterface move_group_interface("left_arm");
		move_group_interface.setEndEffector("left_gripper");

		move_group_interface.setStartStateToCurrentState();


		std::vector<geometry_msgs::PoseStamped> pose_points;

		//orientation of the gripper should always be pointed down
		tf2::Quaternion orientation;
		orientation.setRPY(0.0, M_PI / 2, 0.0);
		geometry_msgs::Quaternion down_orientation = tf2::toMsg(orientation);

		//i is height
		for (int i = 0; i < 3; i++)
		{	
			//j is width
			for (int j = 0; j < 3; j++)
			{
				//I want to say that I personally despise different coordinate systems
				geometry_msgs::PoseStamped n_pos;
				n_pos.pose.position.x = start_x + x_offset - (2 * this->height_between_dots - i * this->height_between_dots);
				n_pos.pose.position.y = start_y + y_offset - j * this->width_between_dots;
				n_pos.pose.position.z = this->z_height;

				n_pos.pose.orientation = down_orientation;

				n_pos.header.frame_id = "base_link";

				pose_points.push_back(n_pos);

				//ROS_INFO("Point[i, j] is at: [%f, %f]", n_pos.pose.position.x, n_pos.pose.position.y);
			}
		}

		moveit_msgs::Constraints constraints;
		moveit_msgs::OrientationConstraint pen_pointed_down;

		pen_pointed_down.header.frame_id = "base_link";

	    pen_pointed_down.link_name = move_group_interface.getEndEffectorLink();
	    pen_pointed_down.orientation = down_orientation;

	    pen_pointed_down.absolute_x_axis_tolerance = 0.45;
	    pen_pointed_down.absolute_y_axis_tolerance = 3.6;
	    pen_pointed_down.absolute_z_axis_tolerance = 0.45;

	    pen_pointed_down.weight = 1;



		std::vector<std::vector<int>> draw_coords;
		int num_vectors = 0;

		this->local_.getParam(this->draw_char + "/strokes", num_vectors);

		for (int i = 0; i < num_vectors; i++)
		{
			std::vector<int> n_coords;
			this->local_.getParam(this->draw_char + "/_" + std::to_string(i + 1), n_coords);
			draw_coords.push_back(n_coords);
		}
  		moveit::planning_interface::MoveGroupInterface::Plan my_plan;

		for (std::vector<int> coords: draw_coords)
		{
			move_group_interface.setPathConstraints(constraints);
			for (int i: coords)
			{
				ROS_INFO("setting goal target [%d], at [%f, %f]", i, pose_points[i - 1].pose.position.x, pose_points[i - 1].pose.position.y);
				move_group_interface.setPoseTarget(pose_points[i - 1], "l_wrist_roll_link");

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
			}

			move_group_interface.clearPathConstraints();

			//pick up the pen
			geometry_msgs::PoseStamped lift_pose = move_group_interface.getCurrentPose();

			lift_pose.pose.position.z = this->pick_up_height;

			move_group_interface.setPoseTarget(lift_pose, "l_wrist_roll_link");

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
		}


		y_offset = y_offset - this->width_to_next;

		if (start_y + y_offset - 2 * this->width_between_dots <= this->min_y)
		{
			x_offset = x_offset - this->height_to_next;
			y_offset = 0.0;
		}

		this->local_.setParam("x_offset", x_offset);
		this->local_.setParam("y_offset", y_offset);

		ROS_WARN("Work Finished for: [%s]", this->name_->topic.c_str());
	}
}
