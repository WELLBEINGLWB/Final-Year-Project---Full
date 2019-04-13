// Generated by gencpp from file segmentation/pathPlannerResponse.msg
// DO NOT EDIT!


#ifndef SEGMENTATION_MESSAGE_PATHPLANNERRESPONSE_H
#define SEGMENTATION_MESSAGE_PATHPLANNERRESPONSE_H


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
struct pathPlannerResponse_
{
  typedef pathPlannerResponse_<ContainerAllocator> Type;

  pathPlannerResponse_()
    : path_found(false)  {
    }
  pathPlannerResponse_(const ContainerAllocator& _alloc)
    : path_found(false)  {
  (void)_alloc;
    }



   typedef uint8_t _path_found_type;
  _path_found_type path_found;




  typedef boost::shared_ptr< ::segmentation::pathPlannerResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::segmentation::pathPlannerResponse_<ContainerAllocator> const> ConstPtr;

}; // struct pathPlannerResponse_

typedef ::segmentation::pathPlannerResponse_<std::allocator<void> > pathPlannerResponse;

typedef boost::shared_ptr< ::segmentation::pathPlannerResponse > pathPlannerResponsePtr;
typedef boost::shared_ptr< ::segmentation::pathPlannerResponse const> pathPlannerResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::segmentation::pathPlannerResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::segmentation::pathPlannerResponse_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::segmentation::pathPlannerResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::segmentation::pathPlannerResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::segmentation::pathPlannerResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::segmentation::pathPlannerResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::segmentation::pathPlannerResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::segmentation::pathPlannerResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::segmentation::pathPlannerResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d5550685e8802ccb55d7b325d3ef4459";
  }

  static const char* value(const ::segmentation::pathPlannerResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd5550685e8802ccbULL;
  static const uint64_t static_value2 = 0x55d7b325d3ef4459ULL;
};

template<class ContainerAllocator>
struct DataType< ::segmentation::pathPlannerResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "segmentation/pathPlannerResponse";
  }

  static const char* value(const ::segmentation::pathPlannerResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::segmentation::pathPlannerResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool path_found\n\
\n\
";
  }

  static const char* value(const ::segmentation::pathPlannerResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::segmentation::pathPlannerResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.path_found);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct pathPlannerResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::segmentation::pathPlannerResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::segmentation::pathPlannerResponse_<ContainerAllocator>& v)
  {
    s << indent << "path_found: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.path_found);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SEGMENTATION_MESSAGE_PATHPLANNERRESPONSE_H