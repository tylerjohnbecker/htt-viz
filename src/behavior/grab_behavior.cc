#include "behavior/grab_behavior.h"

namespace task_net {
	void GrabBehavior::Grab(float width)
	{
		ROS_WARN("Work started for: [%s]", this->name_->topic.c_str());

		//code responsible for grabbing here
		ros::Duration(5).sleep();

		ROS_WARN("Work Finished for: [%s]", this->name_->topic.c_str());
	}
}
