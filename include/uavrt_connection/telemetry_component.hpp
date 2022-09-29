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

// The format of the symbol name should be <PROJECT>_<PATH>_<FILE>_HPP_.
// https://google.github.io/styleguide/cppguide.html#The__define_Guard
#ifndef UAVRT_CONNECTION_INCLUDE_UAVRT_CONNECTION_TELEMETRY_COMPONENT_HPP_
#define UAVRT_CONNECTION_INCLUDE_UAVRT_CONNECTION_TELEMETRY_COMPONENT_HPP_

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

// Boost header files
#include "boost/qvm/quat.hpp"
#include "boost/qvm/quat_operations.hpp"

// Project header files
#include "uavrt_interfaces/msg/pulse.hpp"
#include "uavrt_interfaces/msg/pulse_pose.hpp"

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

struct InterpolationResults {
	double interpolated_latitude;
	double interpolated_logitude;
	double interpolated_altitude;
	boost::qvm::quat<float> interpolated_quaternion;
};

// Create the Telemetry component object by inheriting from rclcpp::Node.
// Note: The component objects in this package don't contain a (virtual)
// deconstructor since it is not expected that the compenents be inherited from.
class TelemetryComponent : public rclcpp::Node
{
public:
explicit TelemetryComponent(const rclcpp::NodeOptions &options,
                            std::shared_ptr<mavsdk::System> system);

private:
// ROS 2 related - Private functions
void AntennaPoseCallback();

void PulseCallback(uavrt_interfaces::msg::Pulse::SharedPtr pulse_message);

// MAVSDK related - Private functions
void ConnectionCallback(bool is_connected);

void PositionCallback(mavsdk::Telemetry::Position position);
void QuaternionCallback(mavsdk::Telemetry::Quaternion quaternion);

// Private helper functions
InterpolationResults InterpolatePosition(double pulse_average_time);

// ROS 2 related - Private variables
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr antenna_pose_publisher_;

rclcpp::Subscription<uavrt_interfaces::msg::Pulse>::SharedPtr pulse_subscriber_;

rclcpp::Publisher<uavrt_interfaces::msg::PulsePose>::SharedPtr pulse_pose_publisher_;

int queue_size_ = 10;

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

// For use in antenna pose callback
std_msgs::msg::Header antenna_pose_header_;
geometry_msgs::msg::PoseStamped antenna_pose_pose_stamped_;
geometry_msgs::msg::Pose antenna_pose_pose_;
geometry_msgs::msg::Point antenna_pose_point_;
geometry_msgs::msg::Quaternion antenna_pose_quaternion_;

// For use in pulse callback
geometry_msgs::msg::Pose pulse_pose_;
geometry_msgs::msg::Point pulse_point_;
geometry_msgs::msg::Quaternion pulse_quaternion_;
uavrt_interfaces::msg::PulsePose pulse_pulse_pose_;

// MAVSDK related - Private variables
mavsdk::Telemetry mavsdk_telemetry;

bool connection_status_ = true;

// Note: Value represent Hertz, 2 Hz = 0.5 seconds
uint8_t postion_subscribe_rate_ = 2;
uint8_t attitude_subscribe_rate_ = 2;

// Info on array of vectors: https://www.geeksforgeeks.org/array-of-vectors-in-c-stl/
// Make a 3 by 1 array of vectors expecting to be filled with type double
// Note: Latitude and Longitude are doubles, but Altitude is a float. We eat
// the extra memory cost to avoid using std::any/std::variant.
// [[0] - Latitude
// [0] - Longitude
// [0]] - Altitude
std::vector<double> antenna_pose_position_array_[3];

// Make a 4 by 1 array of vectors expecting to be filled with type float.
// [[0] - w
// [0] - x
// [0] - y
// [0]] - z
std::vector<float> antenna_pose_quaternion_array_[4];

// Make a vector that expects to be filled with type double.
// Float was not large enough to store these values.
// No reason to make this an array of vectors since we store the time as
// seconds + nanoseconds for easy lookup later. If this needs to be broken up
// into separate parts, then follow the same scheme as above and change the
// "vector" of the name to array.
std::vector<double> antenna_pose_time_vector_;

// https://mavsdk.mavlink.io/main/en/cpp/api_reference/structmavsdk_1_1_telemetry_1_1_position.html#structmavsdk_1_1_telemetry_1_1_position_1
// Latitude in degrees (range: -90 to +90)
// Longitude in degrees (range: -180 to +180)
// Altitude AMSL (above mean sea level) in metres
mavsdk::Telemetry::Position position_;

// https://mavsdk.mavlink.io/main/en/cpp/api_reference/structmavsdk_1_1_telemetry_1_1_quaternion.html
mavsdk::Telemetry::Quaternion quaternion_;

};

}  // namespace uavrt_connection

#endif  // UAVRT_CONNECTION_INCLUDE_UAVRT_CONNECTION_TELEMETRY_COMPONENT_HPP_
