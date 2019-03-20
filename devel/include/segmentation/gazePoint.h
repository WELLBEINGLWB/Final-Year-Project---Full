// Generated by gencpp from file segmentation/gazePoint.msg
// DO NOT EDIT!


#ifndef SEGMENTATION_MESSAGE_GAZEPOINT_H
#define SEGMENTATION_MESSAGE_GAZEPOINT_H

#include <ros/service_traits.h>


#include <segmentation/gazePointRequest.h>
#include <segmentation/gazePointResponse.h>


namespace segmentation
{

struct gazePoint
{

typedef gazePointRequest Request;
typedef gazePointResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct gazePoint
} // namespace segmentation


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::segmentation::gazePoint > {
  static const char* value()
  {
    return "775e18c60bb3bbf62cc8777df14f5d02";
  }

  static const char* value(const ::segmentation::gazePoint&) { return value(); }
};

template<>
struct DataType< ::segmentation::gazePoint > {
  static const char* value()
  {
    return "segmentation/gazePoint";
  }

  static const char* value(const ::segmentation::gazePoint&) { return value(); }
};


// service_traits::MD5Sum< ::segmentation::gazePointRequest> should match 
// service_traits::MD5Sum< ::segmentation::gazePoint > 
template<>
struct MD5Sum< ::segmentation::gazePointRequest>
{
  static const char* value()
  {
    return MD5Sum< ::segmentation::gazePoint >::value();
  }
  static const char* value(const ::segmentation::gazePointRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::segmentation::gazePointRequest> should match 
// service_traits::DataType< ::segmentation::gazePoint > 
template<>
struct DataType< ::segmentation::gazePointRequest>
{
  static const char* value()
  {
    return DataType< ::segmentation::gazePoint >::value();
  }
  static const char* value(const ::segmentation::gazePointRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::segmentation::gazePointResponse> should match 
// service_traits::MD5Sum< ::segmentation::gazePoint > 
template<>
struct MD5Sum< ::segmentation::gazePointResponse>
{
  static const char* value()
  {
    return MD5Sum< ::segmentation::gazePoint >::value();
  }
  static const char* value(const ::segmentation::gazePointResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::segmentation::gazePointResponse> should match 
// service_traits::DataType< ::segmentation::gazePoint > 
template<>
struct DataType< ::segmentation::gazePointResponse>
{
  static const char* value()
  {
    return DataType< ::segmentation::gazePoint >::value();
  }
  static const char* value(const ::segmentation::gazePointResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // SEGMENTATION_MESSAGE_GAZEPOINT_H