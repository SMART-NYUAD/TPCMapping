// Generated by gencpp from file ptu_control/pan_tiltResponse.msg
// DO NOT EDIT!


#ifndef PTU_CONTROL_MESSAGE_PAN_TILTRESPONSE_H
#define PTU_CONTROL_MESSAGE_PAN_TILTRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace ptu_control
{
template <class ContainerAllocator>
struct pan_tiltResponse_
{
  typedef pan_tiltResponse_<ContainerAllocator> Type;

  pan_tiltResponse_()
    : success(false)  {
    }
  pan_tiltResponse_(const ContainerAllocator& _alloc)
    : success(false)  {
  (void)_alloc;
    }



   typedef uint8_t _success_type;
  _success_type success;





  typedef boost::shared_ptr< ::ptu_control::pan_tiltResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ptu_control::pan_tiltResponse_<ContainerAllocator> const> ConstPtr;

}; // struct pan_tiltResponse_

typedef ::ptu_control::pan_tiltResponse_<std::allocator<void> > pan_tiltResponse;

typedef boost::shared_ptr< ::ptu_control::pan_tiltResponse > pan_tiltResponsePtr;
typedef boost::shared_ptr< ::ptu_control::pan_tiltResponse const> pan_tiltResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ptu_control::pan_tiltResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ptu_control::pan_tiltResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ptu_control::pan_tiltResponse_<ContainerAllocator1> & lhs, const ::ptu_control::pan_tiltResponse_<ContainerAllocator2> & rhs)
{
  return lhs.success == rhs.success;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ptu_control::pan_tiltResponse_<ContainerAllocator1> & lhs, const ::ptu_control::pan_tiltResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ptu_control

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::ptu_control::pan_tiltResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ptu_control::pan_tiltResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ptu_control::pan_tiltResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ptu_control::pan_tiltResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ptu_control::pan_tiltResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ptu_control::pan_tiltResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ptu_control::pan_tiltResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "358e233cde0c8a8bcfea4ce193f8fc15";
  }

  static const char* value(const ::ptu_control::pan_tiltResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x358e233cde0c8a8bULL;
  static const uint64_t static_value2 = 0xcfea4ce193f8fc15ULL;
};

template<class ContainerAllocator>
struct DataType< ::ptu_control::pan_tiltResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ptu_control/pan_tiltResponse";
  }

  static const char* value(const ::ptu_control::pan_tiltResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ptu_control::pan_tiltResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool success\n"
;
  }

  static const char* value(const ::ptu_control::pan_tiltResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ptu_control::pan_tiltResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.success);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct pan_tiltResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ptu_control::pan_tiltResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ptu_control::pan_tiltResponse_<ContainerAllocator>& v)
  {
    s << indent << "success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.success);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PTU_CONTROL_MESSAGE_PAN_TILTRESPONSE_H
