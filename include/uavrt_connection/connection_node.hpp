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
#ifndef UAVRT_CONNECTION_INCLUDE_UAVRT_CONNECTION_CONNECTION_NODE_H_
#define UAVRT_CONNECTION_INCLUDE_UAVRT_CONNECTION_CONNECTION_NODE_H_

// Note: Header files from other packages/libraries have to be added to
// uavrt_connection/package.xml and/or uavrt_connection/CMakeLists.txt
// ROS 2 header files
#include "rclcpp/rclcpp.hpp"

// ROS 2 interface header files
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

// MAVSDK header files
#include "mavsdk/mavsdk.h"
#include "mavsdk/system.h"

// Project header files
#include "uavrt_interfaces/msg/pulse.hpp"
#include "uavrt_interfaces/msg/pulse_pose.hpp"
#include "uavrt_interfaces/msg/tag_def.hpp"

#include "uavrt_connection/telemetry_handler.hpp"
#include "uavrt_connection/command_handler.hpp"

namespace uavrt_connection
{

enum class PositionArray
{
	LatitudeIndex = 0,
	LongitudeIndex = 1,
	AltitudeIndex = 2
};

enum class QuaternionArray
{
	WIndex = 0,
	XIndex = 1,
    YIndex = 2,
    ZIndex = 3
};

// Create the Connection object by inheriting from rclcpp::Node.
// Note: This Connection Node doesn't contain a (virtual) deconstructor since
// it is not expected to be inherited from.
class ConnectionNode : public rclcpp::Node
{
public:
explicit ConnectionNode(const rclcpp::NodeOptions &options,
                        std::shared_ptr<mavsdk::System> system);

private:
// Private member functions - ROS 2 related
void AntennaPoseCallback();
void DetectedPulseCallback(uavrt_interfaces::msg::Pulse::SharedPtr pulse_message);

// Private member functions - MAVSDK related
void ConnectionCallback(bool is_connected);

// Private member variables - ROS 2 related
int queue_size_ = 10;

rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr antenna_pose_publisher_;
rclcpp::Publisher<uavrt_interfaces::msg::TagDef>::SharedPtr tag_publisher_;

rclcpp::Subscription<uavrt_interfaces::msg::Pulse>::SharedPtr pulse_subscriber_;

// Info on array of vectors: https://www.geeksforgeeks.org/array-of-vectors-in-c-stl/
// Make a 3 by 1 array of vectors expecting to be filled with type double
// Note: Latitude and Longitude are doubles, but Altitude is a float. We eat
// the extra memory cost to avoid using std::any/std::variant.
// [[0] - Latitude
// [0] - Longitude
// [0]] - Altitude
std::vector<double> position_array_[3];

// Make a 4 by 1 array of vectors expecting to be filled with type float.
// [[0] - w
// [0] - x
// [0] - y
// [0]] - z
std::vector<float> quaternion_array_[4];

std_msgs::msg::Header antenna_pose_header_;
geometry_msgs::msg::PoseStamped antenna_pose_pose_stamped_;
geometry_msgs::msg::Pose antenna_pose_pose_;
geometry_msgs::msg::Point antenna_pose_point_;
geometry_msgs::msg::Quaternion antenna_pose_quaternion_;

// Class template std::chrono::duration represents a time interval.
// Utilizes a repetition and a ratio (using std::ratio).
// https://en.cppreference.com/w/cpp/chrono/duration
// Note: The following line is equivalent to the one being used...
// static constexpr auto period_ms_ = std::literals::chrono_literals::operator""ms(500);
static constexpr auto antenna_pose_period_ms_ = std::chrono::milliseconds(500);

// From what I can tell, this is a shared_ptr to a rclcpp::TimerBase object,
// which contains a shared_ptr to a rclcpp::Clock object.
// https://github.com/ros2/rclcpp/blob/galactic/rclcpp/include/rclcpp/timer.hpp
// https://github.com/ros2/rclcpp/blob/galactic/rclcpp/src/rclcpp/clock.cpp
// https://github.com/ros2/rclcpp/blob/galactic/rclcpp/src/rclcpp/time_source.cpp
rclcpp::TimerBase::SharedPtr antenna_pose_timer_;

// Private member variables - MAVSDK related
bool connection_status_;

// Private member objects
TelemetryHandler TelemetryHandlerObject;
CommandHandler CommandHandlerObject;
};

}  // namespace uavrt_connection

#endif  // UAVRT_CONNECTION_INCLUDE_UAVRT_CONNECTION_CONNECTION_NODE_H_
