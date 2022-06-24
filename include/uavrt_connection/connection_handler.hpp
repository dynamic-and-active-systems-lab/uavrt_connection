// The format of the symbol name should be <PROJECT>_<PATH>_<FILE>_H_.
// https://google.github.io/styleguide/cppguide.html#The__define_Guard
#ifndef UAVRT_CONNECTION_INCLUDE_UAVRT_CONNECTION_CONNECTION_HANDLER_H_
#define UAVRT_CONNECTION_INCLUDE_UAVRT_CONNECTION_CONNECTION_HANDLER_H_

// ROS 2 header files
#include "rclcpp/rclcpp.hpp"

namespace uavrt_connection
{

class Connection : public rclcpp::Node
{
public:
explicit Connection(const rclcpp::NodeOptions &options);

protected:
void TelemetryCallback();

private:

// Class template std::chrono::duration represents a time interval.
// Utilizes a repetition and a ratio (using std::ratio).
// https://en.cppreference.com/w/cpp/chrono/duration
// Note: The following line is equivalent to the one being used...
// static constexpr auto period_ms_ = std::literals::chrono_literals::operator""ms(500);
static constexpr auto telemetry_period_ms_ = std::chrono::milliseconds(500);

rclcpp::TimerBase::SharedPtr telemetry_timer_;
};

}  // namespace uavrt_connection

#endif  // UAVRT_CONNECTION_INCLUDE_UAVRT_CONNECTION_CONNECTION_HANDLER_H_
