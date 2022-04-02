#include "behavior/draw_behavior.h"

namespace task_net {
	void DrawBehavior::Work()
	{
		ROS_WARN("Work started for: [%s]", this->name_->topic.c_str());

		double x_offset, y_offset;
		this->local_.getParam("x_offset", x_offset);
		this->local_.getParam("y_offset", y_offset);

		if (y_offset >= this->max_y)
		{
			ROS_INFO("There is no space to draw so please erase the board...");
			//std::cin;

			x_offset = 0.0;
			y_offset = 0.0;
			ROS_INFO("Starting again from [x, y]: [%f, %f]", x_offset, y_offset); 
		}

		ROS_INFO("Drawing at [x, y]: [%f, %f]", x_offset, y_offset);

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
				geometry_msgs::PoseStamped n_pos;
				n_pos.pose.position.x = x_offset + j * this->width_between_dots;
				n_pos.pose.position.y = y_offset + (2 * this->height_between_dots - i * this->height_between_dots);
				n_pos.pose.position.z = this->z_height;

				n_pos.pose.orientation = down_orientation;

				pose_points.push_back(n_pos);
			}
		}

		std::vector<std::vector<int>> draw_coords;
		int num_vectors = 0;

		this->local_.getParam(this->draw_char + "/strokes", num_vectors);

		for (int i = 0; i < num_vectors; i++)
		{
			std::vector<int> n_coords;
			this->local_.getParam(this->draw_char + "/_" + std::to_string(i + 1), n_coords);
			draw_coords.push_back(n_coords);
		}

		for (std::vector<int> coords: draw_coords)
		{
			ROS_INFO("Printing...");
			for (int i: coords)
				ROS_INFO("i[%d]", i);
		}


		x_offset = x_offset + this->width_to_next;

		if (x_offset >= this->max_x)
		{
			y_offset = y_offset + this->height_to_next;
			x_offset = 0.0;
		}

		this->local_.setParam("x_offset", x_offset);
		this->local_.setParam("y_offset", y_offset);

		ROS_WARN("Work Finished for: [%s]", this->name_->topic.c_str());
	}
}
