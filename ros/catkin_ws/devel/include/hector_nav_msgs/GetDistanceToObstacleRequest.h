// Generated by gencpp from file hector_nav_msgs/GetDistanceToObstacleRequest.msg
// DO NOT EDIT!


#ifndef HECTOR_NAV_MSGS_MESSAGE_GETDISTANCETOOBSTACLEREQUEST_H
#define HECTOR_NAV_MSGS_MESSAGE_GETDISTANCETOOBSTACLEREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/PointStamped.h>

namespace hector_nav_msgs
{
template <class ContainerAllocator>
struct GetDistanceToObstacleRequest_
{
  typedef GetDistanceToObstacleRequest_<ContainerAllocator> Type;

  GetDistanceToObstacleRequest_()
    : point()  {
    }
  GetDistanceToObstacleRequest_(const ContainerAllocator& _alloc)
    : point(_alloc)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::PointStamped_<ContainerAllocator>  _point_type;
  _point_type point;





  typedef boost::shared_ptr< ::hector_nav_msgs::GetDistanceToObstacleRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hector_nav_msgs::GetDistanceToObstacleRequest_<ContainerAllocator> const> ConstPtr;

}; // struct GetDistanceToObstacleRequest_

typedef ::hector_nav_msgs::GetDistanceToObstacleRequest_<std::allocator<void> > GetDistanceToObstacleRequest;

typedef boost::shared_ptr< ::hector_nav_msgs::GetDistanceToObstacleRequest > GetDistanceToObstacleRequestPtr;
typedef boost::shared_ptr< ::hector_nav_msgs::GetDistanceToObstacleRequest const> GetDistanceToObstacleRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hector_nav_msgs::GetDistanceToObstacleRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hector_nav_msgs::GetDistanceToObstacleRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hector_nav_msgs::GetDistanceToObstacleRequest_<ContainerAllocator1> & lhs, const ::hector_nav_msgs::GetDistanceToObstacleRequest_<ContainerAllocator2> & rhs)
{
  return lhs.point == rhs.point;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hector_nav_msgs::GetDistanceToObstacleRequest_<ContainerAllocator1> & lhs, const ::hector_nav_msgs::GetDistanceToObstacleRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hector_nav_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::hector_nav_msgs::GetDistanceToObstacleRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hector_nav_msgs::GetDistanceToObstacleRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hector_nav_msgs::GetDistanceToObstacleRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hector_nav_msgs::GetDistanceToObstacleRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hector_nav_msgs::GetDistanceToObstacleRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hector_nav_msgs::GetDistanceToObstacleRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hector_nav_msgs::GetDistanceToObstacleRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "47dfdbd810b48d0a47b7ad67e4191bcc";
  }

  static const char* value(const ::hector_nav_msgs::GetDistanceToObstacleRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x47dfdbd810b48d0aULL;
  static const uint64_t static_value2 = 0x47b7ad67e4191bccULL;
};

template<class ContainerAllocator>
struct DataType< ::hector_nav_msgs::GetDistanceToObstacleRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hector_nav_msgs/GetDistanceToObstacleRequest";
  }

  static const char* value(const ::hector_nav_msgs::GetDistanceToObstacleRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hector_nav_msgs::GetDistanceToObstacleRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"\n"
"\n"
"\n"
"\n"
"geometry_msgs/PointStamped point\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/PointStamped\n"
"# This represents a Point with reference coordinate frame and timestamp\n"
"Header header\n"
"Point point\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::hector_nav_msgs::GetDistanceToObstacleRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hector_nav_msgs::GetDistanceToObstacleRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.point);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetDistanceToObstacleRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hector_nav_msgs::GetDistanceToObstacleRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hector_nav_msgs::GetDistanceToObstacleRequest_<ContainerAllocator>& v)
  {
    s << indent << "point: ";
    s << std::endl;
    Printer< ::geometry_msgs::PointStamped_<ContainerAllocator> >::stream(s, indent + "  ", v.point);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HECTOR_NAV_MSGS_MESSAGE_GETDISTANCETOOBSTACLEREQUEST_H
