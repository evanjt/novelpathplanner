// Generated by gencpp from file webots_ros/display_image_loadResponse.msg
// DO NOT EDIT!


#ifndef WEBOTS_ROS_MESSAGE_DISPLAY_IMAGE_LOADRESPONSE_H
#define WEBOTS_ROS_MESSAGE_DISPLAY_IMAGE_LOADRESPONSE_H


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
struct display_image_loadResponse_
{
  typedef display_image_loadResponse_<ContainerAllocator> Type;

  display_image_loadResponse_()
    : ir(0)  {
    }
  display_image_loadResponse_(const ContainerAllocator& _alloc)
    : ir(0)  {
  (void)_alloc;
    }



   typedef uint64_t _ir_type;
  _ir_type ir;





  typedef boost::shared_ptr< ::webots_ros::display_image_loadResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::webots_ros::display_image_loadResponse_<ContainerAllocator> const> ConstPtr;

}; // struct display_image_loadResponse_

typedef ::webots_ros::display_image_loadResponse_<std::allocator<void> > display_image_loadResponse;

typedef boost::shared_ptr< ::webots_ros::display_image_loadResponse > display_image_loadResponsePtr;
typedef boost::shared_ptr< ::webots_ros::display_image_loadResponse const> display_image_loadResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::webots_ros::display_image_loadResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::webots_ros::display_image_loadResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace webots_ros

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsMessage': True, 'IsFixedSize': True, 'HasHeader': False}
// {'webots_ros': ['/home/evan/catkin_ws/src/webots_ros/msg'], 'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/melodic/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/melodic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__dir__', '__doc__', '__eq__', '__format__', '__ge__', '__getattribute__', '__gt__', '__hash__', '__init__', '__init_subclass__', '__le__', '__lt__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsMessage< ::webots_ros::display_image_loadResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::display_image_loadResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::webots_ros::display_image_loadResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::webots_ros::display_image_loadResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::display_image_loadResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::display_image_loadResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::webots_ros::display_image_loadResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "062bd6ec8c99fd70e30cc24256f9226a";
  }

  static const char* value(const ::webots_ros::display_image_loadResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x062bd6ec8c99fd70ULL;
  static const uint64_t static_value2 = 0xe30cc24256f9226aULL;
};

template<class ContainerAllocator>
struct DataType< ::webots_ros::display_image_loadResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "webots_ros/display_image_loadResponse";
  }

  static const char* value(const ::webots_ros::display_image_loadResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::webots_ros::display_image_loadResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint64 ir\n"
"\n"
;
  }

  static const char* value(const ::webots_ros::display_image_loadResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::webots_ros::display_image_loadResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.ir);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct display_image_loadResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::webots_ros::display_image_loadResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::webots_ros::display_image_loadResponse_<ContainerAllocator>& v)
  {
    s << indent << "ir: ";
    Printer<uint64_t>::stream(s, indent + "  ", v.ir);
  }
};

} // namespace message_operations
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_DISPLAY_IMAGE_LOADRESPONSE_H
