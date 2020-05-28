// Generated by gencpp from file webots_ros/node_get_fieldRequest.msg
// DO NOT EDIT!


#ifndef WEBOTS_ROS_MESSAGE_NODE_GET_FIELDREQUEST_H
#define WEBOTS_ROS_MESSAGE_NODE_GET_FIELDREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace webots_ros
{
template <class ContainerAllocator>
struct node_get_fieldRequest_
{
  typedef node_get_fieldRequest_<ContainerAllocator> Type;

  node_get_fieldRequest_()
    : node(0)
    , fieldName()  {
    }
  node_get_fieldRequest_(const ContainerAllocator& _alloc)
    : node(0)
    , fieldName(_alloc)  {
  (void)_alloc;
    }



   typedef uint64_t _node_type;
  _node_type node;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _fieldName_type;
  _fieldName_type fieldName;





  typedef boost::shared_ptr< ::webots_ros::node_get_fieldRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::webots_ros::node_get_fieldRequest_<ContainerAllocator> const> ConstPtr;

}; // struct node_get_fieldRequest_

typedef ::webots_ros::node_get_fieldRequest_<std::allocator<void> > node_get_fieldRequest;

typedef boost::shared_ptr< ::webots_ros::node_get_fieldRequest > node_get_fieldRequestPtr;
typedef boost::shared_ptr< ::webots_ros::node_get_fieldRequest const> node_get_fieldRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::webots_ros::node_get_fieldRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::webots_ros::node_get_fieldRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::webots_ros::node_get_fieldRequest_<ContainerAllocator1> & lhs, const ::webots_ros::node_get_fieldRequest_<ContainerAllocator2> & rhs)
{
  return lhs.node == rhs.node &&
    lhs.fieldName == rhs.fieldName;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::webots_ros::node_get_fieldRequest_<ContainerAllocator1> & lhs, const ::webots_ros::node_get_fieldRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace webots_ros

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::webots_ros::node_get_fieldRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::node_get_fieldRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::webots_ros::node_get_fieldRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::webots_ros::node_get_fieldRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::node_get_fieldRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::node_get_fieldRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::webots_ros::node_get_fieldRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2d4687ce20fc4847078a8ce0c31e1e6b";
  }

  static const char* value(const ::webots_ros::node_get_fieldRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2d4687ce20fc4847ULL;
  static const uint64_t static_value2 = 0x078a8ce0c31e1e6bULL;
};

template<class ContainerAllocator>
struct DataType< ::webots_ros::node_get_fieldRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "webots_ros/node_get_fieldRequest";
  }

  static const char* value(const ::webots_ros::node_get_fieldRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::webots_ros::node_get_fieldRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint64 node\n"
"string fieldName\n"
;
  }

  static const char* value(const ::webots_ros::node_get_fieldRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::webots_ros::node_get_fieldRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.node);
      stream.next(m.fieldName);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct node_get_fieldRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::webots_ros::node_get_fieldRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::webots_ros::node_get_fieldRequest_<ContainerAllocator>& v)
  {
    s << indent << "node: ";
    Printer<uint64_t>::stream(s, indent + "  ", v.node);
    s << indent << "fieldName: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.fieldName);
  }
};

} // namespace message_operations
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_NODE_GET_FIELDREQUEST_H
