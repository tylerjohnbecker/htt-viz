/**
* Description: This is just a behavior that we can use up until we make behaviors for the robot.
* 
* Created By: Tyler Becker
* Created On: 11/28/21
* Last Updated: 11/28/21
*/

#ifndef DUMMY_BEHAVIOR_H_
#define DUMMY_BEHAVIOR_H_

#include "task_tree/behavior.h"

namespace task_net {

	class DummyBehavior : public Node {
	public:
		DummyBehavior(NodeId_t name, NodeList peers, NodeList children, NodeId_t parent,
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

#endif //DUMMY_BEHAVIOR_H_
