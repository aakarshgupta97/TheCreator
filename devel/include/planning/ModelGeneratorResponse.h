// Generated by gencpp from file planning/ModelGeneratorResponse.msg
// DO NOT EDIT!


#ifndef PLANNING_MESSAGE_MODELGENERATORRESPONSE_H
#define PLANNING_MESSAGE_MODELGENERATORRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace planning
{
template <class ContainerAllocator>
struct ModelGeneratorResponse_
{
  typedef ModelGeneratorResponse_<ContainerAllocator> Type;

  ModelGeneratorResponse_()
    : errorCode(0)
    , blocks()
    , num_layers(0)
    , width(0)
    , height(0)  {
    }
  ModelGeneratorResponse_(const ContainerAllocator& _alloc)
    : errorCode(0)
    , blocks(_alloc)
    , num_layers(0)
    , width(0)
    , height(0)  {
  (void)_alloc;
    }



   typedef int8_t _errorCode_type;
  _errorCode_type errorCode;

   typedef std::vector<int8_t, typename ContainerAllocator::template rebind<int8_t>::other >  _blocks_type;
  _blocks_type blocks;

   typedef int8_t _num_layers_type;
  _num_layers_type num_layers;

   typedef int8_t _width_type;
  _width_type width;

   typedef int8_t _height_type;
  _height_type height;




  typedef boost::shared_ptr< ::planning::ModelGeneratorResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::planning::ModelGeneratorResponse_<ContainerAllocator> const> ConstPtr;

}; // struct ModelGeneratorResponse_

typedef ::planning::ModelGeneratorResponse_<std::allocator<void> > ModelGeneratorResponse;

typedef boost::shared_ptr< ::planning::ModelGeneratorResponse > ModelGeneratorResponsePtr;
typedef boost::shared_ptr< ::planning::ModelGeneratorResponse const> ModelGeneratorResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::planning::ModelGeneratorResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::planning::ModelGeneratorResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace planning

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::planning::ModelGeneratorResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::planning::ModelGeneratorResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::planning::ModelGeneratorResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::planning::ModelGeneratorResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::planning::ModelGeneratorResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::planning::ModelGeneratorResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::planning::ModelGeneratorResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f50b0934fbc5c3c4cf0e84c00604ffe4";
  }

  static const char* value(const ::planning::ModelGeneratorResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf50b0934fbc5c3c4ULL;
  static const uint64_t static_value2 = 0xcf0e84c00604ffe4ULL;
};

template<class ContainerAllocator>
struct DataType< ::planning::ModelGeneratorResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "planning/ModelGeneratorResponse";
  }

  static const char* value(const ::planning::ModelGeneratorResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::planning::ModelGeneratorResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int8 errorCode\n\
int8[] blocks\n\
int8 num_layers\n\
int8 width\n\
int8 height\n\
\n\
";
  }

  static const char* value(const ::planning::ModelGeneratorResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::planning::ModelGeneratorResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.errorCode);
      stream.next(m.blocks);
      stream.next(m.num_layers);
      stream.next(m.width);
      stream.next(m.height);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ModelGeneratorResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::planning::ModelGeneratorResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::planning::ModelGeneratorResponse_<ContainerAllocator>& v)
  {
    s << indent << "errorCode: ";
    Printer<int8_t>::stream(s, indent + "  ", v.errorCode);
    s << indent << "blocks[]" << std::endl;
    for (size_t i = 0; i < v.blocks.size(); ++i)
    {
      s << indent << "  blocks[" << i << "]: ";
      Printer<int8_t>::stream(s, indent + "  ", v.blocks[i]);
    }
    s << indent << "num_layers: ";
    Printer<int8_t>::stream(s, indent + "  ", v.num_layers);
    s << indent << "width: ";
    Printer<int8_t>::stream(s, indent + "  ", v.width);
    s << indent << "height: ";
    Printer<int8_t>::stream(s, indent + "  ", v.height);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PLANNING_MESSAGE_MODELGENERATORRESPONSE_H
