// Generated by gencpp from file webots_ros/speaker_speak.msg
// DO NOT EDIT!


#ifndef WEBOTS_ROS_MESSAGE_SPEAKER_SPEAK_H
#define WEBOTS_ROS_MESSAGE_SPEAKER_SPEAK_H

#include <ros/service_traits.h>


#include <webots_ros/speaker_speakRequest.h>
#include <webots_ros/speaker_speakResponse.h>


namespace webots_ros
{

struct speaker_speak
{

typedef speaker_speakRequest Request;
typedef speaker_speakResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct speaker_speak
} // namespace webots_ros


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::webots_ros::speaker_speak > {
  static const char* value()
  {
    return "bc987d1a1839d594218b7c1d6a77ba5e";
  }

  static const char* value(const ::webots_ros::speaker_speak&) { return value(); }
};

template<>
struct DataType< ::webots_ros::speaker_speak > {
  static const char* value()
  {
    return "webots_ros/speaker_speak";
  }

  static const char* value(const ::webots_ros::speaker_speak&) { return value(); }
};


// service_traits::MD5Sum< ::webots_ros::speaker_speakRequest> should match 
// service_traits::MD5Sum< ::webots_ros::speaker_speak > 
template<>
struct MD5Sum< ::webots_ros::speaker_speakRequest>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::speaker_speak >::value();
  }
  static const char* value(const ::webots_ros::speaker_speakRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::webots_ros::speaker_speakRequest> should match 
// service_traits::DataType< ::webots_ros::speaker_speak > 
template<>
struct DataType< ::webots_ros::speaker_speakRequest>
{
  static const char* value()
  {
    return DataType< ::webots_ros::speaker_speak >::value();
  }
  static const char* value(const ::webots_ros::speaker_speakRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::webots_ros::speaker_speakResponse> should match 
// service_traits::MD5Sum< ::webots_ros::speaker_speak > 
template<>
struct MD5Sum< ::webots_ros::speaker_speakResponse>
{
  static const char* value()
  {
    return MD5Sum< ::webots_ros::speaker_speak >::value();
  }
  static const char* value(const ::webots_ros::speaker_speakResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::webots_ros::speaker_speakResponse> should match 
// service_traits::DataType< ::webots_ros::speaker_speak > 
template<>
struct DataType< ::webots_ros::speaker_speakResponse>
{
  static const char* value()
  {
    return DataType< ::webots_ros::speaker_speak >::value();
  }
  static const char* value(const ::webots_ros::speaker_speakResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_SPEAKER_SPEAK_H
