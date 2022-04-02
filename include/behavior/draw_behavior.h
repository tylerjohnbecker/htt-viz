#ifndef DRAW_BEHAVIOR_H_
#define DRAW_BEHAVIOR_H_

#include "task_tree/behavior.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//x is positive to the robot's right and y is positive starting in front of the robot and coming toward it, z is positive going up
namespace task_net
{   
	
	class DrawBehavior: public Behavior
	{
	public:
		DrawBehavior(NodeId_t name, NodeList peers, NodeList children, NodeId_t parent,
				State_t state,
				std::string object,
				std::string to_draw,
				bool use_local_callback_queue = false,
				boost::posix_time::millisec mtime = boost::posix_time::millisec(50)) :
				draw_char(to_draw),
				Behavior(name, peers, children, parent,
					state,
					object,
					use_local_callback_queue,
					mtime) {};

			virtual void Work();

	protected:

	private:

		std::string draw_char;

		//1 cm^2 between each point
		const double width_between_dots = .01;
		const double height_between_dots = .01;

		//each letter's starting point is 3cm away from the previous b/c its 2 dots and space between for a 3rd 
		const double width_to_next = .03;
		const double height_to_next = .03;

		//defining how far we can go before we have to wrap
		//NOTE: i just kinda eyeballed it and went low. also the y value starts that far away and comes in.
		const double max_x = .6;
		const double max_y = .3;

		//this is whatever height that we measure we need
		const double z_height = 0.0;
	};
}
#endif // DRAW_BEHAVIOR_H_