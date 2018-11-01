// Generated by gencpp from file reflex_msgs2/DistalRotationResponse.msg
// DO NOT EDIT!


#ifndef REFLEX_MSGS2_MESSAGE_DISTALROTATIONRESPONSE_H
#define REFLEX_MSGS2_MESSAGE_DISTALROTATIONRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace reflex_msgs2
{
template <class ContainerAllocator>
struct DistalRotationResponse_
{
  typedef DistalRotationResponse_<ContainerAllocator> Type;

  DistalRotationResponse_()
    : rotation()  {
      rotation.assign(0.0);
  }
  DistalRotationResponse_(const ContainerAllocator& _alloc)
    : rotation()  {
  (void)_alloc;
      rotation.assign(0.0);
  }



   typedef boost::array<float, 3>  _rotation_type;
  _rotation_type rotation;




  typedef boost::shared_ptr< ::reflex_msgs2::DistalRotationResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::reflex_msgs2::DistalRotationResponse_<ContainerAllocator> const> ConstPtr;

}; // struct DistalRotationResponse_

typedef ::reflex_msgs2::DistalRotationResponse_<std::allocator<void> > DistalRotationResponse;

typedef boost::shared_ptr< ::reflex_msgs2::DistalRotationResponse > DistalRotationResponsePtr;
typedef boost::shared_ptr< ::reflex_msgs2::DistalRotationResponse const> DistalRotationResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::reflex_msgs2::DistalRotationResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::reflex_msgs2::DistalRotationResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace reflex_msgs2

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'reflex_msgs2': ['/home/cc/ee106a/fa18/class/ee106a-aab/ros_workspaces/TheCreator/src/reflex-ros-pkg/reflex_msgs2/msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::reflex_msgs2::DistalRotationResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::reflex_msgs2::DistalRotationResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::reflex_msgs2::DistalRotationResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::reflex_msgs2::DistalRotationResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::reflex_msgs2::DistalRotationResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::reflex_msgs2::DistalRotationResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::reflex_msgs2::DistalRotationResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8d2ccfdfc1a5ba6babe40fd5c7c04dee";
  }

  static const char* value(const ::reflex_msgs2::DistalRotationResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8d2ccfdfc1a5ba6bULL;
  static const uint64_t static_value2 = 0xabe40fd5c7c04deeULL;
};

template<class ContainerAllocator>
struct DataType< ::reflex_msgs2::DistalRotationResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "reflex_msgs2/DistalRotationResponse";
  }

  static const char* value(const ::reflex_msgs2::DistalRotationResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::reflex_msgs2::DistalRotationResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32[3] rotation\n\
\n\
";
  }

  static const char* value(const ::reflex_msgs2::DistalRotationResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::reflex_msgs2::DistalRotationResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.rotation);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct DistalRotationResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::reflex_msgs2::DistalRotationResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::reflex_msgs2::DistalRotationResponse_<ContainerAllocator>& v)
  {
    s << indent << "rotation[]" << std::endl;
    for (size_t i = 0; i < v.rotation.size(); ++i)
    {
      s << indent << "  rotation[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.rotation[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // REFLEX_MSGS2_MESSAGE_DISTALROTATIONRESPONSE_H