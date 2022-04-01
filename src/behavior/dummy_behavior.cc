#include "behavior/dummy_behavior.h"

namespace task_net {
	void DummyBehavior::Work()
	{
		ROS_WARN("Work started for: [%s]", this->name_->topic.c_str());

		ros::Duration(5).sleep();

		ROS_WARN("Work Finished for: [%s]", this->name_->topic.c_str());
	}
} 