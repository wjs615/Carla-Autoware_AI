// Generated by gencpp from file carla_msgs/CarlaEgoVehicleInfoWheel.msg
// DO NOT EDIT!


#ifndef CARLA_MSGS_MESSAGE_CARLAEGOVEHICLEINFOWHEEL_H
#define CARLA_MSGS_MESSAGE_CARLAEGOVEHICLEINFOWHEEL_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Vector3.h>

namespace carla_msgs
{
template <class ContainerAllocator>
struct CarlaEgoVehicleInfoWheel_
{
  typedef CarlaEgoVehicleInfoWheel_<ContainerAllocator> Type;

  CarlaEgoVehicleInfoWheel_()
    : tire_friction(0.0)
    , damping_rate(0.0)
    , max_steer_angle(0.0)
    , radius(0.0)
    , max_brake_torque(0.0)
    , max_handbrake_torque(0.0)
    , position()  {
    }
  CarlaEgoVehicleInfoWheel_(const ContainerAllocator& _alloc)
    : tire_friction(0.0)
    , damping_rate(0.0)
    , max_steer_angle(0.0)
    , radius(0.0)
    , max_brake_torque(0.0)
    , max_handbrake_torque(0.0)
    , position(_alloc)  {
  (void)_alloc;
    }



   typedef float _tire_friction_type;
  _tire_friction_type tire_friction;

   typedef float _damping_rate_type;
  _damping_rate_type damping_rate;

   typedef float _max_steer_angle_type;
  _max_steer_angle_type max_steer_angle;

   typedef float _radius_type;
  _radius_type radius;

   typedef float _max_brake_torque_type;
  _max_brake_torque_type max_brake_torque;

   typedef float _max_handbrake_torque_type;
  _max_handbrake_torque_type max_handbrake_torque;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _position_type;
  _position_type position;





  typedef boost::shared_ptr< ::carla_msgs::CarlaEgoVehicleInfoWheel_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::carla_msgs::CarlaEgoVehicleInfoWheel_<ContainerAllocator> const> ConstPtr;

}; // struct CarlaEgoVehicleInfoWheel_

typedef ::carla_msgs::CarlaEgoVehicleInfoWheel_<std::allocator<void> > CarlaEgoVehicleInfoWheel;

typedef boost::shared_ptr< ::carla_msgs::CarlaEgoVehicleInfoWheel > CarlaEgoVehicleInfoWheelPtr;
typedef boost::shared_ptr< ::carla_msgs::CarlaEgoVehicleInfoWheel const> CarlaEgoVehicleInfoWheelConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::carla_msgs::CarlaEgoVehicleInfoWheel_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::carla_msgs::CarlaEgoVehicleInfoWheel_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::carla_msgs::CarlaEgoVehicleInfoWheel_<ContainerAllocator1> & lhs, const ::carla_msgs::CarlaEgoVehicleInfoWheel_<ContainerAllocator2> & rhs)
{
  return lhs.tire_friction == rhs.tire_friction &&
    lhs.damping_rate == rhs.damping_rate &&
    lhs.max_steer_angle == rhs.max_steer_angle &&
    lhs.radius == rhs.radius &&
    lhs.max_brake_torque == rhs.max_brake_torque &&
    lhs.max_handbrake_torque == rhs.max_handbrake_torque &&
    lhs.position == rhs.position;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::carla_msgs::CarlaEgoVehicleInfoWheel_<ContainerAllocator1> & lhs, const ::carla_msgs::CarlaEgoVehicleInfoWheel_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace carla_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::carla_msgs::CarlaEgoVehicleInfoWheel_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::carla_msgs::CarlaEgoVehicleInfoWheel_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::carla_msgs::CarlaEgoVehicleInfoWheel_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::carla_msgs::CarlaEgoVehicleInfoWheel_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::carla_msgs::CarlaEgoVehicleInfoWheel_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::carla_msgs::CarlaEgoVehicleInfoWheel_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::carla_msgs::CarlaEgoVehicleInfoWheel_<ContainerAllocator> >
{
  static const char* value()
  {
    return "192cba6d0621954855f8c957508a83b4";
  }

  static const char* value(const ::carla_msgs::CarlaEgoVehicleInfoWheel_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x192cba6d06219548ULL;
  static const uint64_t static_value2 = 0x55f8c957508a83b4ULL;
};

template<class ContainerAllocator>
struct DataType< ::carla_msgs::CarlaEgoVehicleInfoWheel_<ContainerAllocator> >
{
  static const char* value()
  {
    return "carla_msgs/CarlaEgoVehicleInfoWheel";
  }

  static const char* value(const ::carla_msgs::CarlaEgoVehicleInfoWheel_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::carla_msgs::CarlaEgoVehicleInfoWheel_<ContainerAllocator> >
{
  static const char* value()
  {
    return "#\n"
"# Copyright (c) 2019-2020 Intel Corporation.\n"
"#\n"
"# This work is licensed under the terms of the MIT license.\n"
"# For a copy, see <https://opensource.org/licenses/MIT>.\n"
"#\n"
"float32 tire_friction\n"
"float32 damping_rate\n"
"float32 max_steer_angle\n"
"float32 radius\n"
"float32 max_brake_torque\n"
"float32 max_handbrake_torque\n"
"geometry_msgs/Vector3 position\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Vector3\n"
"# This represents a vector in free space. \n"
"# It is only meant to represent a direction. Therefore, it does not\n"
"# make sense to apply a translation to it (e.g., when applying a \n"
"# generic rigid transformation to a Vector3, tf2 will only apply the\n"
"# rotation). If you want your data to be translatable too, use the\n"
"# geometry_msgs/Point message instead.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::carla_msgs::CarlaEgoVehicleInfoWheel_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::carla_msgs::CarlaEgoVehicleInfoWheel_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.tire_friction);
      stream.next(m.damping_rate);
      stream.next(m.max_steer_angle);
      stream.next(m.radius);
      stream.next(m.max_brake_torque);
      stream.next(m.max_handbrake_torque);
      stream.next(m.position);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CarlaEgoVehicleInfoWheel_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::carla_msgs::CarlaEgoVehicleInfoWheel_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::carla_msgs::CarlaEgoVehicleInfoWheel_<ContainerAllocator>& v)
  {
    s << indent << "tire_friction: ";
    Printer<float>::stream(s, indent + "  ", v.tire_friction);
    s << indent << "damping_rate: ";
    Printer<float>::stream(s, indent + "  ", v.damping_rate);
    s << indent << "max_steer_angle: ";
    Printer<float>::stream(s, indent + "  ", v.max_steer_angle);
    s << indent << "radius: ";
    Printer<float>::stream(s, indent + "  ", v.radius);
    s << indent << "max_brake_torque: ";
    Printer<float>::stream(s, indent + "  ", v.max_brake_torque);
    s << indent << "max_handbrake_torque: ";
    Printer<float>::stream(s, indent + "  ", v.max_handbrake_torque);
    s << indent << "position: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.position);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CARLA_MSGS_MESSAGE_CARLAEGOVEHICLEINFOWHEEL_H
