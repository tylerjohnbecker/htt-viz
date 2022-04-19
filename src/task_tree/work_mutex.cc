#include "task_tree/work_mutex.h"

#include "task_tree/node.h"

namespace task_net
{
	WorkMutex::~WorkMutex()
	{
		if (run_thread != nullptr)
		{
			run_thread->join();
			delete run_thread;
			run_thread = nullptr;
		}
	}

	void WorkMutex::run()
	{
		waitForBidders();
		Auction();
		sendResults();
	}

	void WorkMutex::start_run ()
	{
		if (run_thread != nullptr)
		{
			run_thread->join();
			delete run_thread;
			run_thread = nullptr;
		}

		run_thread = new boost::thread(&WorkMutex::run, this);
	}

	void WorkMutex::waitForBidders()
	{
		accepting_bids = true;

		ros::Duration(5).sleep();

		accepting_bids = false;
	}

	void WorkMutex::Auction ()
	{
		//minus infinity
		float highest = -1 * std::numeric_limits<float>::infinity();

		//auction for the highest activation_potential
		for (const auto& kv: this->bidders)
		{
			if (kv.second->state_.activation_potential > highest)
			{
				winner = kv.first;
				highest = kv.second->state_.activation_potential;
			}
		}

		mut = true;
	}
	
	void WorkMutex::sendResults()
	{
		for (const auto& kv: this->bidders)
		{
			if (!std::strcmp(kv.first.c_str(), this->winner.c_str()))
			{
				//tell the winner to go ahead
				kv.second->mutexNotifier(true);
			}
			else
			{
				//tell the others to go die
				kv.second->mutexNotifier(false);
			}
		}

		//clear bidder list
		this->bidders.clear();
		started = false;
	}

	bool WorkMutex::bid(Node* bidder)
	{	
		if (mut)
			return false;
		else if (!started)
		{
			started = true;
			bidders[bidder->name_->topic] = bidder;

			start_run();

			return true;
		}
		else if (accepting_bids)
		{
			bidders[bidder->name_->topic] = bidder;

			return true;
		}
		
		return false;
	}

	bool WorkMutex::release (std::string name)
	{
		if (!std::strcmp(name.c_str(), this->winner.c_str()))
		{
			mut = false;

			this->winner = "";
			return true;
		}

		return false;
	}
}