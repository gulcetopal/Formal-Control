// Generated by gencpp from file formal_control/SelfStateMsg.msg
// DO NOT EDIT!


#ifndef FORMAL_CONTROL_MESSAGE_SELFSTATEMSG_H
#define FORMAL_CONTROL_MESSAGE_SELFSTATEMSG_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace formal_control
{
template <class ContainerAllocator>
struct SelfStateMsg_
{
  typedef SelfStateMsg_<ContainerAllocator> Type;

  SelfStateMsg_()
    : header()
    , rfdist(0.0)
    , lfdist(0.0)
    , bdist(0.0)
    , v_relative(0.0)
    , actions()
    , policy()
    , old_policy()
    , v_emg(0.0)
    , yaw_ref(0.0)
    , got_new_plan(false)
    , emergency(0)
    , crit_check()
    , current_state(0)
    , lane(0)
    , timestep(0)
    , request(false)  {
    }
  SelfStateMsg_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , rfdist(0.0)
    , lfdist(0.0)
    , bdist(0.0)
    , v_relative(0.0)
    , actions(_alloc)
    , policy(_alloc)
    , old_policy(_alloc)
    , v_emg(0.0)
    , yaw_ref(0.0)
    , got_new_plan(false)
    , emergency(0)
    , crit_check(_alloc)
    , current_state(0)
    , lane(0)
    , timestep(0)
    , request(false)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef float _rfdist_type;
  _rfdist_type rfdist;

   typedef float _lfdist_type;
  _lfdist_type lfdist;

   typedef float _bdist_type;
  _bdist_type bdist;

   typedef float _v_relative_type;
  _v_relative_type v_relative;

   typedef std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  _actions_type;
  _actions_type actions;

   typedef std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  _policy_type;
  _policy_type policy;

   typedef std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  _old_policy_type;
  _old_policy_type old_policy;

   typedef float _v_emg_type;
  _v_emg_type v_emg;

   typedef float _yaw_ref_type;
  _yaw_ref_type yaw_ref;

   typedef uint8_t _got_new_plan_type;
  _got_new_plan_type got_new_plan;

   typedef int32_t _emergency_type;
  _emergency_type emergency;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _crit_check_type;
  _crit_check_type crit_check;

   typedef int32_t _current_state_type;
  _current_state_type current_state;

   typedef int32_t _lane_type;
  _lane_type lane;

   typedef int32_t _timestep_type;
  _timestep_type timestep;

   typedef uint8_t _request_type;
  _request_type request;





  typedef boost::shared_ptr< ::formal_control::SelfStateMsg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::formal_control::SelfStateMsg_<ContainerAllocator> const> ConstPtr;

}; // struct SelfStateMsg_

typedef ::formal_control::SelfStateMsg_<std::allocator<void> > SelfStateMsg;

typedef boost::shared_ptr< ::formal_control::SelfStateMsg > SelfStateMsgPtr;
typedef boost::shared_ptr< ::formal_control::SelfStateMsg const> SelfStateMsgConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::formal_control::SelfStateMsg_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::formal_control::SelfStateMsg_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace formal_control

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'formal_control': ['/home/gulce/final_ws/src/formal_control/msg', '/home/gulce/final_ws/src/formal_control/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::formal_control::SelfStateMsg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::formal_control::SelfStateMsg_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::formal_control::SelfStateMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::formal_control::SelfStateMsg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::formal_control::SelfStateMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::formal_control::SelfStateMsg_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::formal_control::SelfStateMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e16fb09f7f887b94d9bd0e0a250f07a7";
  }

  static const char* value(const ::formal_control::SelfStateMsg_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe16fb09f7f887b94ULL;
  static const uint64_t static_value2 = 0xd9bd0e0a250f07a7ULL;
};

template<class ContainerAllocator>
struct DataType< ::formal_control::SelfStateMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "formal_control/SelfStateMsg";
  }

  static const char* value(const ::formal_control::SelfStateMsg_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::formal_control::SelfStateMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n\
\n\
float32 rfdist\n\
float32 lfdist\n\
float32 bdist\n\
float32 v_relative\n\
\n\
int32[] actions\n\
int32[] policy\n\
int32[] old_policy\n\
\n\
float32 v_emg\n\
float32 yaw_ref\n\
\n\
bool got_new_plan\n\
\n\
int32 emergency\n\
string crit_check\n\
int32 current_state\n\
int32 lane\n\
int32 timestep\n\
bool request\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";
  }

  static const char* value(const ::formal_control::SelfStateMsg_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::formal_control::SelfStateMsg_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.rfdist);
      stream.next(m.lfdist);
      stream.next(m.bdist);
      stream.next(m.v_relative);
      stream.next(m.actions);
      stream.next(m.policy);
      stream.next(m.old_policy);
      stream.next(m.v_emg);
      stream.next(m.yaw_ref);
      stream.next(m.got_new_plan);
      stream.next(m.emergency);
      stream.next(m.crit_check);
      stream.next(m.current_state);
      stream.next(m.lane);
      stream.next(m.timestep);
      stream.next(m.request);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SelfStateMsg_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::formal_control::SelfStateMsg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::formal_control::SelfStateMsg_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "rfdist: ";
    Printer<float>::stream(s, indent + "  ", v.rfdist);
    s << indent << "lfdist: ";
    Printer<float>::stream(s, indent + "  ", v.lfdist);
    s << indent << "bdist: ";
    Printer<float>::stream(s, indent + "  ", v.bdist);
    s << indent << "v_relative: ";
    Printer<float>::stream(s, indent + "  ", v.v_relative);
    s << indent << "actions[]" << std::endl;
    for (size_t i = 0; i < v.actions.size(); ++i)
    {
      s << indent << "  actions[" << i << "]: ";
      Printer<int32_t>::stream(s, indent + "  ", v.actions[i]);
    }
    s << indent << "policy[]" << std::endl;
    for (size_t i = 0; i < v.policy.size(); ++i)
    {
      s << indent << "  policy[" << i << "]: ";
      Printer<int32_t>::stream(s, indent + "  ", v.policy[i]);
    }
    s << indent << "old_policy[]" << std::endl;
    for (size_t i = 0; i < v.old_policy.size(); ++i)
    {
      s << indent << "  old_policy[" << i << "]: ";
      Printer<int32_t>::stream(s, indent + "  ", v.old_policy[i]);
    }
    s << indent << "v_emg: ";
    Printer<float>::stream(s, indent + "  ", v.v_emg);
    s << indent << "yaw_ref: ";
    Printer<float>::stream(s, indent + "  ", v.yaw_ref);
    s << indent << "got_new_plan: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.got_new_plan);
    s << indent << "emergency: ";
    Printer<int32_t>::stream(s, indent + "  ", v.emergency);
    s << indent << "crit_check: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.crit_check);
    s << indent << "current_state: ";
    Printer<int32_t>::stream(s, indent + "  ", v.current_state);
    s << indent << "lane: ";
    Printer<int32_t>::stream(s, indent + "  ", v.lane);
    s << indent << "timestep: ";
    Printer<int32_t>::stream(s, indent + "  ", v.timestep);
    s << indent << "request: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.request);
  }
};

} // namespace message_operations
} // namespace ros

#endif // FORMAL_CONTROL_MESSAGE_SELFSTATEMSG_H
