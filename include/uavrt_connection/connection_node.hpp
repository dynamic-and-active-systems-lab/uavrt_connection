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
#include "mavsdk/plugins/telemetry/telemetry.h"
#include "mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h"

// Project header files
#include "uavrt_connection/telemetry_handler.hpp"
#include "uavrt_connection/command_handler.hpp"

namespace uavrt_connection
{

enum class CommandID
{
    CommandIDAck = 1, // Ack response to command
    CommandIDTag = 2, // Tag info
    CommandIDPulse = 3 // Detected pulse value
};

enum class AckIndex
{
    AckIndexCommand = 0, // Command being acked
    AckIndexResult = 2 // Command result - 1 success, 0 failure
};

// Create the Connection object by inheriting from rclcpp::Node.
// Note: This class doesn't contain a (virtual) deconstructor since it is not
// expected to be inherited from.
class ConnectionNode : public rclcpp::Node
{
public:
explicit ConnectionNode(const rclcpp::NodeOptions &options);

private:
// Private member functions - ROS 2 related
void AntennaPoseCallback();

// Private member functions - MAVSDK related
void GetSystem();
void ConnectionCallback(bool is_connected);
bool CommandCallback(mavlink_message_t& message);

// Private member variables - ROS 2 related
int queue_size_ = 10;

rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr antenna_pose_publisher_;
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
mavsdk::Mavsdk serial_mavsdk_;
mavsdk::Mavsdk udp_mavsdk_;
std::shared_ptr<mavsdk::System> system_;
bool connection_status_;
std::shared_ptr<mavsdk::MavlinkPassthrough> mavlink_passthrough_;

// Private member objects
TelemetryHandler TelemetryHandlerObject;
CommandHandler CommandHandlerObject;
};

}  // namespace uavrt_connection

#endif  // UAVRT_CONNECTION_INCLUDE_UAVRT_CONNECTION_CONNECTION_NODE_H_
