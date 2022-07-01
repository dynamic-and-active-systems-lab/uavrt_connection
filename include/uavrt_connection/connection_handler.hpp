// Codebase for the Connection package used within the UAV-RT architecture.
// Copyright (C) 2022 Dynamic and Active Systems Lab
//
// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of  MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.
//
// You should have received a copy of the GNU General Public License along with
// this program.  If not, see <http://www.gnu.org/licenses/>.

// The format of the symbol name should be <PROJECT>_<PATH>_<FILE>_H_.
// https://google.github.io/styleguide/cppguide.html#The__define_Guard
#ifndef UAVRT_CONNECTION_INCLUDE_UAVRT_CONNECTION_CONNECTION_HANDLER_H_
#define UAVRT_CONNECTION_INCLUDE_UAVRT_CONNECTION_CONNECTION_HANDLER_H_

// ROS 2 header files
#include "rclcpp/rclcpp.hpp"

namespace uavrt_connection
{

// Note: This class doesn't contain a (virtual) deconstructor since it is not
// expected to be inherited from.
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

// From what I can tell, this is a shared_ptr to a rclcpp::TimerBase object,
// which contains a shared_ptr to a rclcpp::Clock object.
// https://github.com/ros2/rclcpp/blob/galactic/rclcpp/include/rclcpp/timer.hpp
// https://github.com/ros2/rclcpp/blob/galactic/rclcpp/src/rclcpp/clock.cpp
// https://github.com/ros2/rclcpp/blob/galactic/rclcpp/src/rclcpp/time_source.cpp
rclcpp::TimerBase::SharedPtr telemetry_timer_;
};

}  // namespace uavrt_connection

#endif  // UAVRT_CONNECTION_INCLUDE_UAVRT_CONNECTION_CONNECTION_HANDLER_H_
