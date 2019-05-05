// Generated by gencpp from file segmentation/executionOrderResponse.msg
// DO NOT EDIT!


#ifndef SEGMENTATION_MESSAGE_EXECUTIONORDERRESPONSE_H
#define SEGMENTATION_MESSAGE_EXECUTIONORDERRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace segmentation
{
template <class ContainerAllocator>
struct executionOrderResponse_
{
  typedef executionOrderResponse_<ContainerAllocator> Type;

  executionOrderResponse_()
    : execution_success(false)  {
    }
  executionOrderResponse_(const ContainerAllocator& _alloc)
    : execution_success(false)  {
  (void)_alloc;
    }



   typedef uint8_t _execution_success_type;
  _execution_success_type execution_success;




  typedef boost::shared_ptr< ::segmentation::executionOrderResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::segmentation::executionOrderResponse_<ContainerAllocator> const> ConstPtr;

}; // struct executionOrderResponse_

typedef ::segmentation::executionOrderResponse_<std::allocator<void> > executionOrderResponse;

typedef boost::shared_ptr< ::segmentation::executionOrderResponse > executionOrderResponsePtr;
typedef boost::shared_ptr< ::segmentation::executionOrderResponse const> executionOrderResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::segmentation::executionOrderResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::segmentation::executionOrderResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace segmentation

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/indigo/share/sensor_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::segmentation::executionOrderResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::segmentation::executionOrderResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::segmentation::executionOrderResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::segmentation::executionOrderResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::segmentation::executionOrderResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::segmentation::executionOrderResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::segmentation::executionOrderResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "0fa97b8c73a5dba72b93645f87382de3";
  }

  static const char* value(const ::segmentation::executionOrderResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x0fa97b8c73a5dba7ULL;
  static const uint64_t static_value2 = 0x2b93645f87382de3ULL;
};

template<class ContainerAllocator>
struct DataType< ::segmentation::executionOrderResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "segmentation/executionOrderResponse";
  }

  static const char* value(const ::segmentation::executionOrderResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::segmentation::executionOrderResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool execution_success\n\
\n\
";
  }

  static const char* value(const ::segmentation::executionOrderResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::segmentation::executionOrderResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.execution_success);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct executionOrderResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::segmentation::executionOrderResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::segmentation::executionOrderResponse_<ContainerAllocator>& v)
  {
    s << indent << "execution_success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.execution_success);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SEGMENTATION_MESSAGE_EXECUTIONORDERRESPONSE_H