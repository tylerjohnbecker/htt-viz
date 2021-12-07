/**
* Description: This behaviour gives the robot a destination to travel to.
* 
* Created By: Philip
* Created On: 11/28/21
* Last Updated: 11/28/21
*/

#ifndef MOVE_BEHAVIOR_H_
#define MOVE_BEHAVIOR_H_

#include "task_tree/behavior.h"

namespace task_net {

	class MoveBehavior : public Behavior {
	public:
		MoveBehavior(NodeId_t name, NodeList peers, NodeList children, NodeId_t parent,
			State_t state,
			std::string object,
			bool use_local_callback_queue = false,
			boost::posix_time::millisec mtime = boost::posix_time::millisec(50)) :
			Behavior(name, peers, children, parent,
				state,
				object,
				use_local_callback_queue,
				mtime) {};

		virtual void Move(float x, float y);
	protected:

	private:
	};
}

#endif //MOVE_BEHAVIOR_H_
