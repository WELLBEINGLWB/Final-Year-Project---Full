// Generated by gencpp from file segmentation/AddTwoInts.msg
// DO NOT EDIT!


#ifndef SEGMENTATION_MESSAGE_ADDTWOINTS_H
#define SEGMENTATION_MESSAGE_ADDTWOINTS_H

#include <ros/service_traits.h>


#include <segmentation/AddTwoIntsRequest.h>
#include <segmentation/AddTwoIntsResponse.h>


namespace segmentation
{

struct AddTwoInts
{

typedef AddTwoIntsRequest Request;
typedef AddTwoIntsResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct AddTwoInts
} // namespace segmentation


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::segmentation::AddTwoInts > {
  static const char* value()
  {
    return "6a2e34150c00229791cc89ff309fff21";
  }

  static const char* value(const ::segmentation::AddTwoInts&) { return value(); }
};

template<>
struct DataType< ::segmentation::AddTwoInts > {
  static const char* value()
  {
    return "segmentation/AddTwoInts";
  }

  static const char* value(const ::segmentation::AddTwoInts&) { return value(); }
};


// service_traits::MD5Sum< ::segmentation::AddTwoIntsRequest> should match 
// service_traits::MD5Sum< ::segmentation::AddTwoInts > 
template<>
struct MD5Sum< ::segmentation::AddTwoIntsRequest>
{
  static const char* value()
  {
    return MD5Sum< ::segmentation::AddTwoInts >::value();
  }
  static const char* value(const ::segmentation::AddTwoIntsRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::segmentation::AddTwoIntsRequest> should match 
// service_traits::DataType< ::segmentation::AddTwoInts > 
template<>
struct DataType< ::segmentation::AddTwoIntsRequest>
{
  static const char* value()
  {
    return DataType< ::segmentation::AddTwoInts >::value();
  }
  static const char* value(const ::segmentation::AddTwoIntsRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::segmentation::AddTwoIntsResponse> should match 
// service_traits::MD5Sum< ::segmentation::AddTwoInts > 
template<>
struct MD5Sum< ::segmentation::AddTwoIntsResponse>
{
  static const char* value()
  {
    return MD5Sum< ::segmentation::AddTwoInts >::value();
  }
  static const char* value(const ::segmentation::AddTwoIntsResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::segmentation::AddTwoIntsResponse> should match 
// service_traits::DataType< ::segmentation::AddTwoInts > 
template<>
struct DataType< ::segmentation::AddTwoIntsResponse>
{
  static const char* value()
  {
    return DataType< ::segmentation::AddTwoInts >::value();
  }
  static const char* value(const ::segmentation::AddTwoIntsResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // SEGMENTATION_MESSAGE_ADDTWOINTS_H
