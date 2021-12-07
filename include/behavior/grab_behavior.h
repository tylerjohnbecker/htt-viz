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

namespace task_net {

	class GrabBehavior : public Behavior {
	public:
		GrabBehavior(NodeId_t name, NodeList peers, NodeList children, NodeId_t parent,
			State_t state,
			std::string object,
			bool use_local_callback_queue = false,
			boost::posix_time::millisec mtime = boost::posix_time::millisec(50)) :
			Behavior(name, peers, children, parent,
				state,
				object,
				use_local_callback_queue,
				mtime) {};

		virtual void Grab(float width);
	protected:

	private:
	};
}

#endif //GRAB_BEHAVIOR_H_
