// The format of the symbol name should be <PROJECT>_<PATH>_<FILE>_H_.
// https://google.github.io/styleguide/cppguide.html#The__define_Guard
#ifndef UAVRT_CONNECTION_INCLUDE_UAVRT_CONNECTION_CONNECTION_COMPONENT_H_
#define UAVRT_CONNECTION_INCLUDE_UAVRT_CONNECTION_CONNECTION_COMPONENT_H_

// ROS 2 header files
#include "rclcpp/rclcpp.hpp"

namespace uavrt_connection
{

class Connection : public rclcpp::Node
{
public:
  explicit Connection(const rclcpp::NodeOptions & options);

protected:
  void timer_callback();

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace uavrt_connection

#endif  // UAVRT_CONNECTION_INCLUDE_UAVRT_CONNECTION_CONNECTION_COMPONENT_H_
