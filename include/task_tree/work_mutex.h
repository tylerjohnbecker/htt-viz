#ifndef WORK_MUTEX_H_
#define WORK_MUTEX_H_

#include <map>
#include <boost/thread/thread.hpp>

namespace task_net{
	
	class Node;

	class WorkMutex
	{
	public:
		WorkMutex() : accepting_bids(true), mut(false), started(false) {};
		~WorkMutex();

		void run ();
		void start_run();
		void waitForBidders();
		void Auction();
		void sendResults();

		bool bid(Node* bidder);
		bool release(std::string name);
	private:
		boost::thread* run_thread;

		bool accepting_bids;
		bool mut;
		bool started;

		std::string winner;

		std::map<std::string, Node*> bidders;
	};
}

#endif // WORK_MUTEX_H_