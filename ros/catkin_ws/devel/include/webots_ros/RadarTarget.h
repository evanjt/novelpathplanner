// Generated by gencpp from file webots_ros/RadarTarget.msg
// DO NOT EDIT!


#ifndef WEBOTS_ROS_MESSAGE_RADARTARGET_H
#define WEBOTS_ROS_MESSAGE_RADARTARGET_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace webots_ros
{
template <class ContainerAllocator>
struct RadarTarget_
{
  typedef RadarTarget_<ContainerAllocator> Type;

  RadarTarget_()
    : header()
    , distance(0.0)
    , receivedPower(0.0)
    , speed(0.0)
    , azimuth(0.0)  {
    }
  RadarTarget_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , distance(0.0)
    , receivedPower(0.0)
    , speed(0.0)
    , azimuth(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef double _distance_type;
  _distance_type distance;

   typedef double _receivedPower_type;
  _receivedPower_type receivedPower;

   typedef double _speed_type;
  _speed_type speed;

   typedef double _azimuth_type;
  _azimuth_type azimuth;





  typedef boost::shared_ptr< ::webots_ros::RadarTarget_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::webots_ros::RadarTarget_<ContainerAllocator> const> ConstPtr;

}; // struct RadarTarget_

typedef ::webots_ros::RadarTarget_<std::allocator<void> > RadarTarget;

typedef boost::shared_ptr< ::webots_ros::RadarTarget > RadarTargetPtr;
typedef boost::shared_ptr< ::webots_ros::RadarTarget const> RadarTargetConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::webots_ros::RadarTarget_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::webots_ros::RadarTarget_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace webots_ros

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsMessage': True, 'IsFixedSize': False, 'HasHeader': True}
// {'webots_ros': ['/home/evan/catkin_ws/src/webots_ros/msg'], 'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/melodic/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/melodic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__dir__', '__doc__', '__eq__', '__format__', '__ge__', '__getattribute__', '__gt__', '__hash__', '__init__', '__init_subclass__', '__le__', '__lt__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsMessage< ::webots_ros::RadarTarget_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::RadarTarget_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::webots_ros::RadarTarget_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::webots_ros::RadarTarget_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::RadarTarget_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::RadarTarget_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::webots_ros::RadarTarget_<ContainerAllocator> >
{
  static const char* value()
  {
    return "39dda2b01810c27987f6a767b1a78c1c";
  }

  static const char* value(const ::webots_ros::RadarTarget_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x39dda2b01810c279ULL;
  static const uint64_t static_value2 = 0x87f6a767b1a78c1cULL;
};

template<class ContainerAllocator>
struct DataType< ::webots_ros::RadarTarget_<ContainerAllocator> >
{
  static const char* value()
  {
    return "webots_ros/RadarTarget";
  }

  static const char* value(const ::webots_ros::RadarTarget_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::webots_ros::RadarTarget_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"float64 distance\n"
"float64 receivedPower\n"
"float64 speed\n"
"float64 azimuth\n"
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
;
  }

  static const char* value(const ::webots_ros::RadarTarget_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::webots_ros::RadarTarget_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.distance);
      stream.next(m.receivedPower);
      stream.next(m.speed);
      stream.next(m.azimuth);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RadarTarget_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::webots_ros::RadarTarget_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::webots_ros::RadarTarget_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "distance: ";
    Printer<double>::stream(s, indent + "  ", v.distance);
    s << indent << "receivedPower: ";
    Printer<double>::stream(s, indent + "  ", v.receivedPower);
    s << indent << "speed: ";
    Printer<double>::stream(s, indent + "  ", v.speed);
    s << indent << "azimuth: ";
    Printer<double>::stream(s, indent + "  ", v.azimuth);
  }
};

} // namespace message_operations
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_RADARTARGET_H
