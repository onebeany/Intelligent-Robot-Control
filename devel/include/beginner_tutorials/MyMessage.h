// Generated by gencpp from file beginner_tutorials/MyMessage.msg
// DO NOT EDIT!


#ifndef BEGINNER_TUTORIALS_MESSAGE_MYMESSAGE_H
#define BEGINNER_TUTORIALS_MESSAGE_MYMESSAGE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace beginner_tutorials
{
template <class ContainerAllocator>
struct MyMessage_
{
  typedef MyMessage_<ContainerAllocator> Type;

  MyMessage_()
    : time()
    , nums()  {
    }
  MyMessage_(const ContainerAllocator& _alloc)
    : time(_alloc)
    , nums(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _time_type;
  _time_type time;

   typedef std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> _nums_type;
  _nums_type nums;





  typedef boost::shared_ptr< ::beginner_tutorials::MyMessage_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::beginner_tutorials::MyMessage_<ContainerAllocator> const> ConstPtr;

}; // struct MyMessage_

typedef ::beginner_tutorials::MyMessage_<std::allocator<void> > MyMessage;

typedef boost::shared_ptr< ::beginner_tutorials::MyMessage > MyMessagePtr;
typedef boost::shared_ptr< ::beginner_tutorials::MyMessage const> MyMessageConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::beginner_tutorials::MyMessage_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::beginner_tutorials::MyMessage_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::beginner_tutorials::MyMessage_<ContainerAllocator1> & lhs, const ::beginner_tutorials::MyMessage_<ContainerAllocator2> & rhs)
{
  return lhs.time == rhs.time &&
    lhs.nums == rhs.nums;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::beginner_tutorials::MyMessage_<ContainerAllocator1> & lhs, const ::beginner_tutorials::MyMessage_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace beginner_tutorials

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::beginner_tutorials::MyMessage_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::beginner_tutorials::MyMessage_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::beginner_tutorials::MyMessage_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::beginner_tutorials::MyMessage_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::beginner_tutorials::MyMessage_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::beginner_tutorials::MyMessage_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::beginner_tutorials::MyMessage_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c0c0cdfb81e8e5ff82904cd630ef5b88";
  }

  static const char* value(const ::beginner_tutorials::MyMessage_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc0c0cdfb81e8e5ffULL;
  static const uint64_t static_value2 = 0x82904cd630ef5b88ULL;
};

template<class ContainerAllocator>
struct DataType< ::beginner_tutorials::MyMessage_<ContainerAllocator> >
{
  static const char* value()
  {
    return "beginner_tutorials/MyMessage";
  }

  static const char* value(const ::beginner_tutorials::MyMessage_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::beginner_tutorials::MyMessage_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string time\n"
"int32[] nums\n"
;
  }

  static const char* value(const ::beginner_tutorials::MyMessage_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::beginner_tutorials::MyMessage_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.time);
      stream.next(m.nums);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MyMessage_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::beginner_tutorials::MyMessage_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::beginner_tutorials::MyMessage_<ContainerAllocator>& v)
  {
    s << indent << "time: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.time);
    s << indent << "nums[]" << std::endl;
    for (size_t i = 0; i < v.nums.size(); ++i)
    {
      s << indent << "  nums[" << i << "]: ";
      Printer<int32_t>::stream(s, indent + "  ", v.nums[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // BEGINNER_TUTORIALS_MESSAGE_MYMESSAGE_H
