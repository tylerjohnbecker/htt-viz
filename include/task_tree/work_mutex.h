#ifndef WORK_MUTEX_H_
#define WORK_MUTEX_H_

#include <limits>
#include <boost/thread/thread.hpp>
#include <map>

namespace task_net
{

	class Node;

	class WorkMutex
	{
	public:
		WorkMutex() : started(false), accepting_bids(false), mut(false), run_thread(nullptr) {};
		~WorkMutex();

		void waitForBidders();
		void Auction ();
		void sendResults();

		bool bid(Node* bidder);
		bool release(std::string name);
	private:
		bool started;
		bool accepting_bids;

		bool mut;

		std::map<std::string, Node*> bidders;
		std::string winner;

		boost::thread* run_thread;

		void start_run(); 
		void run();
	};

}

#endif //WORK_MUTEX_H_