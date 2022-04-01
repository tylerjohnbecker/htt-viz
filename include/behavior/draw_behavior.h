#ifndef DRAW_BEHAVIOR_H_
#define DRAW_BEHAVIOR_H_

#include "task_tree/behavior.h"

namespace task_net
{
	class DrawBehavior: public Behavior
	{
		DrawBehavior(NodeId_t name, NodeList peers, NodeList children, NodeId_t parent,
				State_t state,
				std::string object,
				std::string to_draw,
				bool use_local_callback_queue = false,
				boost::posix_time::millisec mtime = boost::posix_time::millisec(50)) :
				Behavior(name, peers, children, parent,
					state,
					object,
					use_local_callback_queue,
					mtime) {};

			virtual void Work();

		protected:

		private:
	};
}
#endif // DRAW_BEHAVIOR_H_