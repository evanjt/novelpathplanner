// Generated by gencpp from file webots_ros/skin_get_bone_orientationResponse.msg
// DO NOT EDIT!


#ifndef WEBOTS_ROS_MESSAGE_SKIN_GET_BONE_ORIENTATIONRESPONSE_H
#define WEBOTS_ROS_MESSAGE_SKIN_GET_BONE_ORIENTATIONRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Quaternion.h>

namespace webots_ros
{
template <class ContainerAllocator>
struct skin_get_bone_orientationResponse_
{
  typedef skin_get_bone_orientationResponse_<ContainerAllocator> Type;

  skin_get_bone_orientationResponse_()
    : orientation()  {
    }
  skin_get_bone_orientationResponse_(const ContainerAllocator& _alloc)
    : orientation(_alloc)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::Quaternion_<ContainerAllocator>  _orientation_type;
  _orientation_type orientation;





  typedef boost::shared_ptr< ::webots_ros::skin_get_bone_orientationResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::webots_ros::skin_get_bone_orientationResponse_<ContainerAllocator> const> ConstPtr;

}; // struct skin_get_bone_orientationResponse_

typedef ::webots_ros::skin_get_bone_orientationResponse_<std::allocator<void> > skin_get_bone_orientationResponse;

typedef boost::shared_ptr< ::webots_ros::skin_get_bone_orientationResponse > skin_get_bone_orientationResponsePtr;
typedef boost::shared_ptr< ::webots_ros::skin_get_bone_orientationResponse const> skin_get_bone_orientationResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::webots_ros::skin_get_bone_orientationResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::webots_ros::skin_get_bone_orientationResponse_<ContainerAllocator> >::stream(s, "", v);
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
struct IsMessage< ::webots_ros::skin_get_bone_orientationResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::skin_get_bone_orientationResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::webots_ros::skin_get_bone_orientationResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::webots_ros::skin_get_bone_orientationResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::skin_get_bone_orientationResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::skin_get_bone_orientationResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::webots_ros::skin_get_bone_orientationResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8ed1de3b69473461225107f55ad59b9d";
  }

  static const char* value(const ::webots_ros::skin_get_bone_orientationResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8ed1de3b69473461ULL;
  static const uint64_t static_value2 = 0x225107f55ad59b9dULL;
};

template<class ContainerAllocator>
struct DataType< ::webots_ros::skin_get_bone_orientationResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "webots_ros/skin_get_bone_orientationResponse";
  }

  static const char* value(const ::webots_ros::skin_get_bone_orientationResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::webots_ros::skin_get_bone_orientationResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "geometry_msgs/Quaternion orientation\n"
"\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
;
  }

  static const char* value(const ::webots_ros::skin_get_bone_orientationResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::webots_ros::skin_get_bone_orientationResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.orientation);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct skin_get_bone_orientationResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::webots_ros::skin_get_bone_orientationResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::webots_ros::skin_get_bone_orientationResponse_<ContainerAllocator>& v)
  {
    s << indent << "orientation: ";
    s << std::endl;
    Printer< ::geometry_msgs::Quaternion_<ContainerAllocator> >::stream(s, indent + "  ", v.orientation);
  }
};

} // namespace message_operations
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_SKIN_GET_BONE_ORIENTATIONRESPONSE_H
