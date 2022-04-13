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
#ifndef NODE_TYPES_H_
#define NODE_TYPES_H_

#include <ros/message_traits.h>
#include <ros/serialization.h>
#include <assert.h>
#include <stdint.h>
#include <string>
#include <vector>
//#include <visualization_msgs/Marker.h>
#include "htt_viz/State.h"
#include "htt_viz/ControlMessage.h"
#include "htt_viz/hold_status.h"
#include "htt_viz/SimState.h"
#include "htt_viz/PeerSimState.h"
#include "htt_viz/ActuatorRequest.h"
#include "htt_viz/ActuatorState.h"



namespace task_net {

static int n; 

typedef enum {  // Four possible task node types
  ROOT = 0,     // 0
  THEN,           // 1
  OR,          // 2
  AND     // 3
} NodeTypes_t;

typedef enum {  // Eight possible robots
  PR2 = 0, // 0
  BAXTER,  // 1
} RobotTypes;

struct NodeBitmask {
  uint8_t type;
  uint8_t robot;
  uint16_t node;
};

struct Issue {
  std::string issue;
  std::string object;
  uint8_t robot_id; 
};
struct Object{

  std::string name; 
  int32_t type;

  geometry_msgs::Pose pose;
  geometry_msgs::Vector3 scale;
  std_msgs::ColorRGBA color;

};
struct Robot{
  geometry_msgs::Pose pose;
  geometry_msgs::Pose goal;

   std_msgs::ColorRGBA color;
   std::string holding;
};

// struct SimState {
//   std::vector<Object> objects;
//   std::vector<Robot> robots;
// };

struct BitmaskLessThan {
    bool operator()(const NodeBitmask& l, const NodeBitmask& r) const {
        return *reinterpret_cast<const uint32_t*>(&l) <
            *reinterpret_cast<const uint32_t*>(&r);
    } ;
};

struct State {
  NodeBitmask owner;  // If owner is null node is inactive
  bool active;
  bool done;
  bool check_peer;
  bool peer_okay;
  float activation_level;
  float activation_potential;
  bool peer_active;
  bool peer_done;
  NodeBitmask highest;
  float highest_potential;
  int parent_type;
  float suitability;
  bool collision;
  bool robotPlacing;
  bool humanPlacing;
  bool peerUndone;
  std::string simstate_obj_name;
  geometry_msgs::Pose simstate_obj_pose;
  geometry_msgs::Pose simstate_robot_pose;
  geometry_msgs::Pose simstate_robot_goal;
  //std::string peer_issue;
};

typedef State State_t;

struct NodeId {
  std::string topic;
  NodeBitmask mask;
  ros::Publisher * pub;
  State_t state;
};
typedef NodeId NodeId_t;


struct ControlMessage {
  NodeBitmask sender;
  int type;
  float activation_level;
  float activation_potential;
  bool done;
  bool active;
  NodeBitmask highest;
  int parent_type;
  bool collision;
  bool robotPlacing;
  bool humanPlacing;
  bool peerUndone;
  std::string simstate_obj_name;
  geometry_msgs::Pose simstate_obj_pose;
  uint8_t simstate_robot_id;
  geometry_msgs::Pose simstate_robot_pose;
  geometry_msgs::Pose simstate_robot_goal;
  std::vector<htt_viz::Actuator> actuators_to_retain;
  std::vector<htt_viz::ActuatorRequest> requests;
  htt_viz::ActuatorState state;
  //std::string peer_issue;

  

};

struct hold_status {
   bool dropped;
   bool pick;
   std::string object_name;
   std::string issue;
};



typedef std::vector<NodeId_t> NodeList;
typedef std::vector<NodeId_t>::iterator NodeListIterator;
typedef std::vector<NodeId_t*> NodeListPtr;
typedef std::vector<NodeId_t*>::iterator NodeListPtrIterator;
typedef std::vector<ros::Publisher> PubList;
typedef ControlMessage ControlMessage_t;
}  // namespace task_net

ROS_STATIC_ASSERT(sizeof(task_net::State_t) == sizeof(task_net::State));
ROS_STATIC_ASSERT(sizeof(task_net::NodeBitmask) ==
  sizeof(task_net::NodeBitmask));

#define is_eq(a,b) (a.robot==b.robot&&a.type==b.type&&a.node==b.node)

namespace ros {
namespace message_traits {
// This type is fixed-size (24-bytes)
template<> struct IsFixedSize<task_net::NodeBitmask> : public TrueType {};
// This type is memcpyable
template<> struct IsSimple<task_net::NodeBitmask> : public TrueType {};

template<>
struct MD5Sum<task_net::NodeBitmask> {
  static const char* value() {
    return MD5Sum<htt_viz::NodeBitmask>::value();
  }

  static const char* value(const task_net::NodeBitmask& m) {
    return MD5Sum<htt_viz::NodeBitmask>::value();
  }
};

template<>
struct DataType<task_net::NodeBitmask> {
  static const char* value() {
    return DataType<htt_viz::NodeBitmask>::value();
  }

  static const char* value(const task_net::NodeBitmask& m) {
    return DataType<htt_viz::NodeBitmask>::value();
  }
};

template<>
struct Definition<task_net::NodeBitmask> {
  static const char* value() {
    return Definition<htt_viz::NodeBitmask>::value();
  }

  static const char* value(const task_net::NodeBitmask& m) {
    return Definition<htt_viz::NodeBitmask>::value();
  }
};

//adding objects 
// template<> struct IsFixedSize<task_net::Issue> : public TrueType {};
// // This type is memcpyable
// template<> struct IsSimple<task_net::Issue> : public TrueType {};

// template<>
// struct MD5Sum<task_net::Issue> {
//   static const char* value() {
//     return MD5Sum<htt_viz::Issue>::value();
//   }

//   static const char* value(const task_net::Issue& m) {
//     return MD5Sum<htt_viz::Issue>::value();
//   }
// };

// template<>
// struct DataType<task_net::Issue> {
//   static const char* value() {
//     return DataType<htt_viz::Issue>::value();
//   }

//   static const char* value(const task_net::Issue& m) {
//     return DataType<htt_viz::Issue>::value();
//   }
// };

// template<>
// struct Definition<task_net::Issue> {
//   static const char* value() {
//     return Definition<htt_viz::Issue>::value();
//   }

//   static const char* value(const task_net::Issue& m) {
//     return Definition<htt_viz::Issue>::value();
//   }
// };

// template<> struct IsFixedSize<task_net::Object> : public TrueType {};
// // This type is memcpyable
// template<> struct IsSimple<task_net::Object> : public TrueType {};

// template<>
// struct MD5Sum<task_net::Object> {
//   static const char* value() {
//     return MD5Sum<htt_viz::Object>::value();
//   }

//   static const char* value(const task_net::Object& m) {
//     return MD5Sum<htt_viz::Object>::value();
//   }
// };

// template<>
// struct DataType<task_net::Object> {
//   static const char* value() {
//     return DataType<htt_viz::Object>::value();
//   }

//   static const char* value(const task_net::Object& m) {
//     return DataType<htt_viz::Object>::value();
//   }
// };

// template<>
// struct Definition<task_net::Object> {
//   static const char* value() {
//     return Definition<htt_viz::Object>::value();
//   }

//   static const char* value(const task_net::Object& m) {
//     return Definition<htt_viz::Object>::value();
//   }
// };

// //adding robots
// template<> struct IsFixedSize<task_net::Robot> : public TrueType {};
// // This type is memcpyable
// template<> struct IsSimple<task_net::Robot> : public TrueType {};

// template<>
// struct MD5Sum<task_net::Robot> {
//   static const char* value() {
//     return MD5Sum<htt_viz::Robot>::value();
//   }

//   static const char* value(const task_net::Robot& m) {
//     return MD5Sum<htt_viz::Robot>::value();
//   }
// };

// template<>
// struct DataType<task_net::Robot> {
//   static const char* value() {
//     return DataType<htt_viz::Robot>::value();
//   }

//   static const char* value(const task_net::Robot& m) {
//     return DataType<htt_viz::Robot>::value();
//   }
// };

// template<>
// struct Definition<task_net::Robot> {
//   static const char* value() {
//     return Definition<htt_viz::Robot>::value();
//   }

//   static const char* value(const task_net::Robot& m) {
//     return Definition<htt_viz::Robot>::value();
//   }
// };

// //adding simstate

// template<> struct IsFixedSize<task_net::SimState> : public TrueType {};
// // This type is memcpyable
// template<> struct IsSimple<task_net::SimState> : public TrueType {};

// template<>
// struct MD5Sum<task_net::SimState> {
//   static const char* value() {
//     return MD5Sum<htt_viz::SimState>::value();
//   }

//   static const char* value(const task_net::SimState& m) {
//     return MD5Sum<htt_viz::SimState>::value();
//   }
// };

// template<>
// struct DataType<task_net::SimState> {
//   static const char* value() {
//     return DataType<htt_viz::SimState>::value();
//   }

//   static const char* value(const task_net::SimState& m) {
//     return DataType<htt_viz::SimState>::value();
//   }
// };

// template<>
// struct Definition<task_net::SimState> {
//   static const char* value() {
//     return Definition<htt_viz::SimState>::value();
//   }

//   static const char* value(const task_net::SimState& m) {
//     return Definition<htt_viz::SimState>::value();
//   }
// };


// This type is fixed-size (24-bytes)
template<> struct IsFixedSize<task_net::State_t> : public TrueType {};
// This type is memcpyable
template<> struct IsSimple<task_net::State_t> : public TrueType {};

template<>
struct MD5Sum<task_net::State_t> {
  static const char* value() {
    return MD5Sum<htt_viz::State>::value();
  }

  static const char* value(const task_net::State_t& m) {
    return MD5Sum<htt_viz::State>::value();
  }
};

template<>
struct DataType<task_net::State_t> {
  static const char* value() {
    return DataType<htt_viz::State>::value();
  }

  static const char* value(const task_net::State_t& m) {
    return DataType<htt_viz::State>::value();
  }
};

template<>
struct Definition<task_net::State_t> {
  static const char* value() {
    return Definition<htt_viz::State>::value();
  }

  static const char* value(const task_net::State_t& m) {
    return Definition<htt_viz::State>::value();
  }
};

////////////////////////////////////////////////////////////////////////////////
// Control Message Conversion for struct type agreement
////////////////////////////////////////////////////////////////////////////////
template<> struct IsFixedSize<task_net::ControlMessage_t> : public TrueType {};
// This type is memcpyable
template<> struct IsSimple<task_net::ControlMessage_t> : public TrueType {};

template<>
struct MD5Sum<task_net::ControlMessage_t> {
  static const char* value() {
    return MD5Sum<htt_viz::ControlMessage>::value();
  }

  static const char* value(const task_net::ControlMessage_t& m) {
    return MD5Sum<htt_viz::ControlMessage>::value();
  }
};

template<>
struct DataType<task_net::ControlMessage_t> {
  static const char* value() {
    return DataType<htt_viz::ControlMessage>::value();
  }

  static const char* value(const task_net::ControlMessage_t& m) {
    return DataType<htt_viz::ControlMessage>::value();
  }
};

template<>
struct Definition<task_net::ControlMessage_t> {
  static const char* value() {
    return Definition<htt_viz::ControlMessage>::value();
  }

  static const char* value(const task_net::ControlMessage_t& m) {
    return Definition<htt_viz::ControlMessage>::value();
  }
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
}  // namespace message_traits

/*TODO: KEEP UP TO DATE WITH STRUCT*/
namespace serialization {
template<>
struct Serializer<task_net::State_t> {
  template<typename Stream, typename T>
  inline static void allInOne(Stream& stream, T t) {
    stream.next(t.owner);
    stream.next(t.active);
    stream.next(t.done);
    stream.next(t.activation_level);
    stream.next(t.activation_potential);
    stream.next(t.peer_active);
    stream.next(t.peer_done);
    stream.next(t.parent_type);
    stream.next(t.suitability);
    stream.next(t.collision);
    stream.next(t.robotPlacing);
    stream.next(t.humanPlacing);
    stream.next(t.peerUndone);
    stream.next(t.simstate_obj_name);
    stream.next(t.simstate_obj_pose);
    stream.next(t.simstate_robot_pose);
    stream.next(t.simstate_robot_goal);
    //stream.next(t.peer_issue);

  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
};

template<>
struct Serializer<task_net::NodeBitmask> {
  template<typename Stream, typename T>
  inline static void allInOne(Stream& stream, T t) {
    stream.next(t.type);
    stream.next(t.robot);
    stream.next(t.node);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
};
// adding issue
// template<>
// struct Serializer<task_net::Issue> {
//   template<typename Stream, typename T>
//   inline static void allInOne(Stream& stream, T t) {
//     stream.next(t.issue);
//     stream.next(t.object);
//     stream.next(t.robot_id);
//   }

//   ROS_DECLARE_ALLINONE_SERIALIZER;
// };




// template<>
// struct Serializer<task_net::Object> {
//   template<typename Stream, typename T>
//   inline static void allInOne(Stream& stream, T t) {
//     stream.next(t.name);
//     stream.next(t.type);
//     stream.next(t.pose);
//     stream.next(t.scale);
//     stream.next(t.color);
//   }

//   ROS_DECLARE_ALLINONE_SERIALIZER;
// };

// template<>
// struct Serializer<task_net::Robot> {
//   template<typename Stream, typename T>
//   inline static void allInOne(Stream& stream, T t) {
//     stream.next(t.pose);
//     stream.next(t.goal);
//     stream.next(t.color);
//     stream.next(t.holding);
//   }

//   ROS_DECLARE_ALLINONE_SERIALIZER;
// };

// template<>
// struct Serializer<task_net::SimState> {
//   template<typename Stream, typename T>
//   inline static void allInOne(Stream& stream, T t) {
//     stream.next(t.objects);
//     stream.next(t.robots);
//   }

//   ROS_DECLARE_ALLINONE_SERIALIZER;
// };
////////////////////////////////////////////////////////////////////////////////
// Control Message Struct Serialization
////////////////////////////////////////////////////////////////////////////////
template<>
struct Serializer<task_net::ControlMessage_t> {
  template<typename Stream, typename T>
  inline static void allInOne(Stream& stream, T t) {
    stream.next(t.sender);
    stream.next(t.type);
    stream.next(t.activation_level);
    stream.next(t.activation_potential);
    stream.next(t.done);
    stream.next(t.active);
    stream.next(t.highest);
    stream.next(t.parent_type);
    stream.next(t.collision);
    stream.next(t.robotPlacing);
    stream.next(t.humanPlacing);
    stream.next(t.peerUndone);
    stream.next(t.simstate_obj_name);
    stream.next(t.simstate_obj_pose);
    stream.next(t.simstate_robot_id);
    stream.next(t.simstate_robot_pose);
    stream.next(t.simstate_robot_goal);
    stream.next(t.actuators_to_retain);
    stream.next(t.requests);
    stream.next(t.state);
    //stream.next(t.peer_issue);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
};
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
}  // namespace serialization
}  // namespace ros
#endif
