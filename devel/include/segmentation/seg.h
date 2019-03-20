// Generated by gencpp from file segmentation/seg.msg
// DO NOT EDIT!


#ifndef SEGMENTATION_MESSAGE_SEG_H
#define SEGMENTATION_MESSAGE_SEG_H

#include <ros/service_traits.h>


#include <segmentation/segRequest.h>
#include <segmentation/segResponse.h>


namespace segmentation
{

struct seg
{

typedef segRequest Request;
typedef segResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct seg
} // namespace segmentation


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::segmentation::seg > {
  static const char* value()
  {
    return "1c5220d340f84f5a98f995334a2a3fe5";
  }

  static const char* value(const ::segmentation::seg&) { return value(); }
};

template<>
struct DataType< ::segmentation::seg > {
  static const char* value()
  {
    return "segmentation/seg";
  }

  static const char* value(const ::segmentation::seg&) { return value(); }
};


// service_traits::MD5Sum< ::segmentation::segRequest> should match 
// service_traits::MD5Sum< ::segmentation::seg > 
template<>
struct MD5Sum< ::segmentation::segRequest>
{
  static const char* value()
  {
    return MD5Sum< ::segmentation::seg >::value();
  }
  static const char* value(const ::segmentation::segRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::segmentation::segRequest> should match 
// service_traits::DataType< ::segmentation::seg > 
template<>
struct DataType< ::segmentation::segRequest>
{
  static const char* value()
  {
    return DataType< ::segmentation::seg >::value();
  }
  static const char* value(const ::segmentation::segRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::segmentation::segResponse> should match 
// service_traits::MD5Sum< ::segmentation::seg > 
template<>
struct MD5Sum< ::segmentation::segResponse>
{
  static const char* value()
  {
    return MD5Sum< ::segmentation::seg >::value();
  }
  static const char* value(const ::segmentation::segResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::segmentation::segResponse> should match 
// service_traits::DataType< ::segmentation::seg > 
template<>
struct DataType< ::segmentation::segResponse>
{
  static const char* value()
  {
    return DataType< ::segmentation::seg >::value();
  }
  static const char* value(const ::segmentation::segResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // SEGMENTATION_MESSAGE_SEG_H