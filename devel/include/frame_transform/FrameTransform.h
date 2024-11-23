// Generated by gencpp from file frame_transform/FrameTransform.msg
// DO NOT EDIT!


#ifndef FRAME_TRANSFORM_MESSAGE_FRAMETRANSFORM_H
#define FRAME_TRANSFORM_MESSAGE_FRAMETRANSFORM_H

#include <ros/service_traits.h>


#include <frame_transform/FrameTransformRequest.h>
#include <frame_transform/FrameTransformResponse.h>


namespace frame_transform
{

struct FrameTransform
{

typedef FrameTransformRequest Request;
typedef FrameTransformResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct FrameTransform
} // namespace frame_transform


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::frame_transform::FrameTransform > {
  static const char* value()
  {
    return "1f0c6508a5e0cc15dc2cd25abbd47d9c";
  }

  static const char* value(const ::frame_transform::FrameTransform&) { return value(); }
};

template<>
struct DataType< ::frame_transform::FrameTransform > {
  static const char* value()
  {
    return "frame_transform/FrameTransform";
  }

  static const char* value(const ::frame_transform::FrameTransform&) { return value(); }
};


// service_traits::MD5Sum< ::frame_transform::FrameTransformRequest> should match
// service_traits::MD5Sum< ::frame_transform::FrameTransform >
template<>
struct MD5Sum< ::frame_transform::FrameTransformRequest>
{
  static const char* value()
  {
    return MD5Sum< ::frame_transform::FrameTransform >::value();
  }
  static const char* value(const ::frame_transform::FrameTransformRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::frame_transform::FrameTransformRequest> should match
// service_traits::DataType< ::frame_transform::FrameTransform >
template<>
struct DataType< ::frame_transform::FrameTransformRequest>
{
  static const char* value()
  {
    return DataType< ::frame_transform::FrameTransform >::value();
  }
  static const char* value(const ::frame_transform::FrameTransformRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::frame_transform::FrameTransformResponse> should match
// service_traits::MD5Sum< ::frame_transform::FrameTransform >
template<>
struct MD5Sum< ::frame_transform::FrameTransformResponse>
{
  static const char* value()
  {
    return MD5Sum< ::frame_transform::FrameTransform >::value();
  }
  static const char* value(const ::frame_transform::FrameTransformResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::frame_transform::FrameTransformResponse> should match
// service_traits::DataType< ::frame_transform::FrameTransform >
template<>
struct DataType< ::frame_transform::FrameTransformResponse>
{
  static const char* value()
  {
    return DataType< ::frame_transform::FrameTransform >::value();
  }
  static const char* value(const ::frame_transform::FrameTransformResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // FRAME_TRANSFORM_MESSAGE_FRAMETRANSFORM_H
