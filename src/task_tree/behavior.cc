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
#include "task_tree/behavior.h"
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <vector>
//#include "remote_mutex/remote_mutex.h"
//#include "geometry_msgs/Pose.h"

namespace task_net {
typedef std::vector<NodeId_t>::iterator NodeId_t_iterator;
// BEHAVIOR
Behavior::Behavior() {}
Behavior::Behavior(NodeId_t name, NodeList peers, NodeList children,
    NodeId_t parent,
    State_t state,
    std::string object,
    bool use_local_callback_queue,
    boost::posix_time::millisec mtime) : Node(name,
      peers,
      children,
      parent,
      state,
      object,
      nullptr) {  
      // printf("Behavior::Behavior WAS CALLED\n");
  ROS_WARN("END OF BEHAVIOR CONSTRUCTOR");
}
Behavior::~Behavior() {}

////////////////////////////////////////////////////////////////////////////////
// AND BEHAVIOR
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
AndBehavior::AndBehavior() {}
AndBehavior::AndBehavior(NodeId_t name, NodeList peers, NodeList children,
    NodeId_t parent,
    State_t state,
    std::string object,
    bool use_local_callback_queue,
    boost::posix_time::millisec mtime) : Behavior(name,
      peers,
      children,
      parent,
      state,
      object) {
        printf("AndBehavior::AndBehavior WAS CALLED\n");
    }
AndBehavior::~AndBehavior() {}

void AndBehavior::UpdateActivationPotential() {
    // ROS_INFO("AndBehavior::UpdateActivationPotential was called!!!!\n");

  // if node is done, activation potential for all children is 0
  if( IsDone() )
  {
    state_.highest_potential = 0;
    state_.highest = mask_;
    return;
  }

  // this should choose bubble up child with highest potential
  float highest = -1;
  NodeBitmask nbm = mask_;
  //ROS_INFO( "default mask (%02d_%1d_%03d)", mask_.type, mask_.robot, mask_.node );
  float sum = 0;
  for (NodeListPtrIterator it = children_.begin();
      it != children_.end(); ++it) {
    sum += (*it)->state.activation_potential;
    if( (*it)->state.activation_potential > highest && !(*it)->state.done && !(*it)->state.peer_done && !(*it)->state.peer_active && !(*it)->state.active )
    {
      //sum += (*it)->state.activation_potential;
      // save as the highest potential
      highest = (*it)->state.activation_potential;
      nbm.type = (*it)->state.highest.type;
      nbm.robot = (*it)->state.highest.robot;
      nbm.node = (*it)->state.highest.node;
      //ROS_INFO( "nbm mask (%02d_%1d_%03d)", nbm.type, nbm.robot, nbm.node );
      //nbm = (*it)->state.highest;
      
    }
  }
  state_.activation_potential = highest;
  //state_.activation_potential = sum /children_.size();
  state_.highest_potential = highest;
  state_.highest.type = nbm.type;
  state_.highest.robot = nbm.robot;
  state_.highest.node = nbm.node;
  //ROS_INFO( "highest mask (%02d_%1d_%03d)", state_.highest.type, state_.highest.robot, state_.highest.node );
}

bool AndBehavior::Precondition() {
    // ROS_INFO("AndBehavior::Precondition was called!!!!\n");
  bool satisfied = true;
  for (NodeListPtrIterator it = children_.begin();
      it != children_.end(); ++it) {
    satisfied = satisfied && (*it)->state.done;
  }
  if (satisfied)
    return true;
  return false;
}

uint32_t AndBehavior::SpreadActivation() {
    // ROS_INFO("AndBehavior::SpreadActivation was called!!!!");

  if (IsDone())
    return 0;

  ControlMessagePtr_t msg(new ControlMessage_t);
  msg->type = 0;
  msg->sender = mask_;
  msg->activation_level = 100.0f / children_.size();
  msg->done = false;

  ROS_DEBUG("Sending activation_level: %f", msg->activation_level);

  for (NodeListPtrIterator it = children_.begin(); it != children_.end();
      ++it) {
    SendToChild((*it)->mask, msg);
  }
}

bool AndBehavior::IsDone() {
  ROS_DEBUG("[%s]: AndBehavior::IsDone was called", name_->topic.c_str() );
  for( int i = 0; i < children_.size(); i++ )
  {
    if( !(children_[i]->state.done || children_[i]->state.peer_done) ) 
    {
      ROS_DEBUG( "[%s]: state not done: %d", name_->topic.c_str(), children_[i]->state.owner.node);
      state_.done = 0;
      return false;
    }
  }

  state_.done = 1;
  return true;
  //return state_.done;
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// THEN BEHAVIOR
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
ThenBehavior::ThenBehavior() {}
ThenBehavior::ThenBehavior(NodeId_t name, NodeList peers, NodeList children,
    NodeId_t parent,
    State_t state,
    std::string object,
    bool use_local_callback_queue,
    boost::posix_time::millisec mtime) : Behavior(name,
      peers,
      children,
      parent,
      state,
      object) {
  // Initialize activation queue
       printf("ThenBehavior::ThenBehavior WAS CALLED\n");
  for (NodeListPtrIterator it = children_.begin(); it != children_.end();
      ++it) {
    activation_queue_.push(*it);
  }
}
ThenBehavior::~ThenBehavior() {}

void ThenBehavior::UpdateActivationPotential() {
  //ROS_INFO("ThenBehavior::UpdateActivationPotential was called!!!!\n");

  // if node is done, activation potential for all children is 0
  if( IsDone() )
  {
    state_.highest_potential = 0;
    state_.highest = mask_;
    return;
  }

  // this should bubble up first not done child
  float highest = 0;
  NodeBitmask nbm = mask_;


  for (NodeListPtrIterator it = children_.begin();
      it != children_.end(); ++it) {
    //ROS_INFO( "\t[%s]: checking [%d]: [%d|%d|%d|%d]", name_->topic.c_str(), (*it)->state.owner.node, (*it)->state.done, (*it)->state.peer_done, (*it)->state.active, (*it)->state.peer_active );
    // find the first not done/active node
    if( !(*it)->state.done && !(*it)->state.peer_done && !(*it)->state.peer_active )
    {
      // save as the highest potential
      highest = (*it)->state.activation_potential;
      nbm = (*it)->state.highest;
      break;
    }
  }

  state_.activation_potential = highest; //sum / children_.size();
  state_.highest_potential = highest;
  state_.highest = nbm;

}

bool ThenBehavior::Precondition() {
    // ROS_INFO("ThenBehavior::Precondition was called!!!!\n");
  bool satisfied = true;
  for (NodeListPtrIterator it = children_.begin();
      it != children_.end(); ++it) {
    satisfied = satisfied && (*it)->state.done;
  }
  if (satisfied)
    return true;
  
  return false;
}

uint32_t ThenBehavior::SpreadActivation() {
  ROS_DEBUG("ThenBehavior::SpreadActivation was called!!!!");
  if (!activation_queue_.empty()) {
    ControlMessagePtr_t msg(new ControlMessage_t);
    msg->type = 0;
    msg->sender = mask_;
    msg->activation_level = 100.0f;
    msg->done = false;

    if (activation_queue_.front()->state.done || activation_queue_.front()->state.peer_done) {
      int old_id = activation_queue_.front()->state.owner.node;
      activation_queue_.pop();
      ROS_INFO( "\t[%s]: node [%d] is done, moving to next node [%d]", name_->topic.c_str(), old_id, activation_queue_.front()->state.owner.node);
    }

    ROS_DEBUG( "[%s]: spreading activation to [%d]", name_->topic.c_str(), activation_queue_.front()->state.owner.node );
    SendToChild(activation_queue_.front()->mask, msg);
  }
}

bool ThenBehavior::IsDone() {
  ROS_DEBUG("[%s]: ThenBehavior::IsDone was called", name_->topic.c_str() );
  for( int i = 0; i < children_.size(); i++ )
  {
    if( !(children_[i]->state.done || children_[i]->state.peer_done) ) 
    {
      ROS_DEBUG( "[%s]: state not done: %d", name_->topic.c_str(), children_[i]->state.owner.node);
      state_.done = 0;
      return false;
    }
  }

  state_.done = 1;
  return true;
  //return state_.done;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// OR BEHAVIOR
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
OrBehavior::OrBehavior() {}
OrBehavior::OrBehavior(NodeId_t name, NodeList peers, NodeList children,
    NodeId_t parent,
    State_t state,
    std::string object,
    bool use_local_callback_queue,
    boost::posix_time::millisec mtime) : Behavior(name,
      peers,
      children,
      parent,
      state,
      object) {
  seed = static_cast<uint32_t>(time(NULL));
  //random_child_selection = rand_r(&seed) % children_.size();
  // printf("OrBehavior::OrBehavior WAS CALLED\n");
}
OrBehavior::~OrBehavior() {}

void OrBehavior::UpdateActivationPotential() {
    // ROS_INFO("OrBehavior::UpdateActivationPotential was called!!!!\n");
  float max = 0;
  int max_child_index = 0, index = 0;

  // this should choose bubble up child with highest potential
  NodeBitmask nbm = mask_;

  // if node is done, activation potential for all children is 0
  if( IsDone() )
  {
    state_.highest_potential = 0;
    state_.highest = mask_;
    return;
  }

  // go thorugh children and get highest activation
  for (NodeListPtrIterator it = children_.begin();
      it != children_.end(); ++it) {
    float value = (*it)->state.activation_potential;

    if((*it)->state.done || (*it)->state.active )
    {
      state_.highest_potential = 0;
      state_.highest = mask_;
      ROS_DEBUG("XXXXXXXXXXXX    CASE 2: me active or done %d XXXXXXXXXXXX", (*it)->mask.node);
      return;
    }
    else if (value > max && !(*it)->state.done && !(*it)->state.active) {
      max = value;
      max_child_index = index;
      nbm = (*it)->state.highest;
      ROS_DEBUG("XXXXXXXXXXXX    CASE 3: value>max  %d XXXXXXXXXXXX", (*it)->mask.node);
    }
    else {
      ROS_DEBUG("XXXXXXXXXXXX    CASE 4: value < max  %d XXXXXXXXXXXX", (*it)->mask.node);
    }
    index++;
  }
  state_.activation_potential = max;
  state_.highest_potential = max;
  state_.highest = nbm;
}

bool OrBehavior::Precondition() {
  ROS_DEBUG("OrBehavior::Precondition was called!!!!\n");
  
  for( int i = 0; i < children_.size(); i++ )
  {
    if( children_[i]->state.done || children_[i]->state.peer_done) 
    {
      ROS_INFO( "[%s]: state done: %d", name_->topic.c_str(), children_[i]->state.owner.node);
      state_.done = 1;
      return true;
    }
  }

  return false;
}

uint32_t OrBehavior::SpreadActivation() {
  ROS_DEBUG("OrBehavior::SpreadActivation was called!!!!");

  if (IsDone())
    return 0;

  ControlMessagePtr_t msg(new ControlMessage_t);
  msg->type = 0;
  msg->sender = mask_;
  msg->activation_level = 100.0f;
  msg->done = this->state_.done;

  for ( int i = 0; i < children_.size(); i++)
  {
    if (node_dict_[children_[i]->mask]->state.collision)
    {
      SendToChild(children_[i]->mask, msg);
      return 0;
    }
  }

  for( int i = 0; i < children_.size(); i++ )
  {
    SendToChild(children_[i]->mask, msg);  
  }
  
}

bool OrBehavior::IsDone() {
  ROS_DEBUG("[%s]: OrBehavior::IsDone was called", name_->topic.c_str() );
  for( int i = 0; i < children_.size(); i++ )
  {
    if( children_[i]->state.done || children_[i]->state.peer_done ) 
    {
      ROS_DEBUG( "[%s]: state done: %d", name_->topic.c_str(), children_[i]->state.owner.node);
      state_.done = 1;
      return true;
    }
  }

  state_.done = false;
  return false;
  //return state_.done;
}


}

