#include "behavior/move_behavior.h"

namespace task_net {
	void MoveBehavior::Work()
	{
		ROS_WARN("Work started for: [%s]", this->name_->topic.c_str());
		
		//movement code here
		ros::Duration(5).sleep();

		ROS_WARN("Work Finished for: [%s]", this->name_->topic.c_str());
	}
}
