/*
robotics-task-tree-eval
Copyright (C) 2015  Luke Fraser

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef INCLUDE_NODE_H_
#define INCLUDE_NODE_H_
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <stdint.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <string>
#include <map>
#include <fstream>
#include <boost/thread/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include "task_tree/node_types.h"
#include "htt_viz/ControlMessage.h"
#include "htt_viz/hold_status.h"
#include "htt_viz/Issue.h"
#include "htt_viz/Resolution.h"
#include "htt_viz/Human_Resolution.h"
#include "htt_viz/SimState.h"
#include "htt_viz/PeerSimState.h"



//#include <pause_pkg/Stop.h>
//typedef htt_viz::hold_status holdPtr;
namespace task_net {

typedef boost::shared_ptr<htt_viz::ControlMessage const>
  ConstControlMessagePtr;
typedef boost::shared_ptr<htt_viz::ControlMessage>
  ControlMessagePtr;
typedef boost::shared_ptr<ControlMessage_t const>
  ConstControlMessagePtr_t;
typedef boost::shared_ptr<ControlMessage_t>
  ControlMessagePtr_t;

// Pre-declare
class Node;
void WorkThread(Node* node);
/*
Class: Node
Definition: Base class for behavior network nodes. All nodes will inherit from
            this class. This list includes AND, THEN, OR, WHILE nodes.
Author: Luke Fraser
*/
class Node {
 public:
  Node();
  Node(NodeId_t name, NodeList peers, NodeList children, NodeId_t parent,
    State_t state,
    std::string object,
    bool use_local_callback_queue = false,
    boost::posix_time::millisec mtime = boost::posix_time::millisec(50));
  virtual ~Node();

  // DFS: tf fix
  virtual void init();

  virtual void undoCallback(ConstControlMessagePtr_t msg);
  virtual void dropCallback(std_msgs::String msg);

  virtual void Update();
  virtual void Work();
  virtual bool CheckWork();
  virtual void UndoWork();
  virtual void PublishStateToPeers();

  

 protected: 
  virtual void Activate();
  virtual void Deactivate();
  virtual void ActivateNode(NodeId_t node);
  virtual void DeactivateNode();
  virtual void DeactivatePeer();
  //virtual void DialogueCallback(const dialogue::Resolution::ConstPtr &msg);
  //virtual void DialogueCallback(const dialogue::Human_Resolution::ConstPtr &msg);
  virtual void DialogueHumanCallback(const htt_viz::Resolution::ConstPtr &msg);
  //virtual void DialogueRobotCallback(const dialogue::Resolution::ConstPtr &msg);
  // virtual void Dialogue();
  //virtual void DialogueRobot();
  virtual void DialogueHuman();
  //virtual void StateCallback(const htt_viz::SimState::ConstPtr& msg);
  virtual void SimStateCallback(htt_viz::SimState msg);

  virtual void Finish();
  virtual State GetState();

  // Messaging
  virtual void SendToParent(
    const htt_viz::ControlMessage msg);
  virtual void SendToParent(const ControlMessagePtr_t msg);
  virtual void SendToChild(NodeBitmask node,
    const htt_viz::ControlMessage msg);
  virtual void SendToChild(NodeBitmask node, const ControlMessagePtr_t msg);
  virtual void SendToPeer(NodeBitmask node,
    const htt_viz::ControlMessage msg);
  virtual void SendToPeer(NodeBitmask node, const ControlMessagePtr_t msg);

  // Receiving Threads
  virtual void ReceiveFromParent(ConstControlMessagePtr_t msg);
  virtual void ReceiveFromChildren(ConstControlMessagePtr_t msg);
  virtual void ReceiveFromPeers(ConstControlMessagePtr_t msg);

  // Main Node loop functions
  virtual bool IsDone();
  virtual bool IsActive();
  virtual float ActivationLevel();
  virtual bool Precondition();
  virtual uint32_t SpreadActivation();
  virtual ros::CallbackQueue* GetPubCallbackQueue();
  virtual ros::CallbackQueue* GetSubCallbackQueue();

  ros::CallbackQueue* pub_callback_queue_;
  ros::CallbackQueue* sub_callback_queue_;


  friend void WorkThread(Node* node);
  friend void RecordThread(Node* node);
  friend void CheckThread(Node* node);
  friend void PeerCheckThread(Node *node);

  virtual void RecordToFile();
 private:
  virtual void NodeInit(boost::posix_time::millisec mtime);
  virtual void PublishStatus();
  virtual void PublishActivationPotential();
  virtual void UpdateActivationPotential();
  virtual void PublishDoneParent();
  virtual void InitializeSubscriber(NodeId_t *node);
  virtual void InitializePublishers(NodeListPtr nodes, PubList *pub,
    const char * topic_addition = "");
  virtual void InitializePublisher(NodeId_t *node, ros::Publisher *pub,
    const char * topic_addition = "");
  virtual void InitializeStatePublisher(NodeId_t *node, ros::Publisher *pub,
    const char * topic_addition = "");
  virtual NodeBitmask GetBitmask(std::string name);
  virtual NodeId_t GetNodeId(NodeBitmask id);
  virtual void GenerateNodeBitmaskMap();
  virtual void InitializeBitmask(NodeId_t* node);
  virtual void InitializeBitmasks(NodeListPtr nodes);
  virtual bool ActivationPrecondition();
  virtual void ActivationFalloff();
  virtual void PublishStateToChildren();

  virtual void ReleaseMutexLocs();
  virtual void releasingRobotNode();

 // to call the vision manip pipeline service
 //ros::ServiceClient* visManipClient_pntr;


 protected:
  std::ofstream record_file;
  NodeId_t *name_;
  State state_;
  htt_viz::hold_status hold_status_;//sd
  bool parent_done_;
  std::map<NodeBitmask, NodeId_t*, BitmaskLessThan> node_dict_;
  std::string name_id_;
  NodeBitmask mask_;
  NodeListPtr peers_;
  NodeListPtr children_;
  NodeId_t *parent_;

  std::string object_;
  //SimState table_state_;
  htt_viz::SimState table_state_;
  Issue peer_issue_;


  // Publishers
  PubList children_pub_list_;
  PubList peer_pub_list_;
  ros::Publisher parent_pub_;
  ros::Publisher self_pub_;
  ros::Publisher undo_pub_;
  ros::Publisher init_dialogue_;
  ros::Publisher peer_simstate_;
  ros::Publisher human_res_pub_;

  // Subscribers
  ros::Subscriber children_sub_;
  ros::Subscriber peer_sub_;
  ros::Subscriber parent_sub_;
  ros::Subscriber undo_sub_;
  ros::Subscriber drop_sub_;
  ros::Subscriber state_sub_ ;

  // service clients
  ros::ServiceClient stopClient_;
  ros::ServiceClient resetClient_;

  // Node handler
  ros::NodeHandle pub_nh_;
  ros::NodeHandle sub_nh_;
  ros::NodeHandle local_;

  // Threads
  boost::thread *update_thread;
  boost::thread *work_thread;
  boost::thread *check_thread;
  boost::thread *peer_check_thread;

  // Mutex
  boost::mutex mut;
  boost::mutex work_mut;
  boost::mutex peer_mut;

  // Recording Mutex
  boost::thread *record_thread;

  // Conditional Variable
  boost::condition_variable cv;

  // Working state
  bool working;
  bool thread_running_;

};
}  // namespace task_net
#endif  // INCLUDE_NODE_H_
