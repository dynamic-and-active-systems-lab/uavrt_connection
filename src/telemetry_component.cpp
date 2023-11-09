// Codebase for the Connection package used within the UAV-RT architecture.
// Copyright (C) 2023 Dynamic and Active Systems Lab
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

// C++ standard library headers
#include <chrono>
#include <functional>
#include <memory>
#include <vector>
#include <tuple>
#include <algorithm>
#include <string>
#include <cmath>
#include <fstream> // Read and write from/to files.

// Delete - only for debugging
#include <iostream>     // std::cout

// Project header files
#include "uavrt_connection/telemetry_component.hpp"

namespace uavrt_connection
{

// Every 'this' in this file is referring to the 'Telemetry component' initialized
// in the member initializer list.
// The Node constructor is passed the name of the node as well as
// applicable options. These are not needed here but are worth considering if
// using parameters.
// https://docs.ros2.org/galactic/api/rclcpp/classrclcpp_1_1NodeOptions.html
TelemetryComponent::TelemetryComponent(const rclcpp::NodeOptions& options,
                                       std::shared_ptr<mavsdk::System> system)
	: Node("TelemetryComponent", options),
	  mavsdk_telemetry(system)
{
	RCLCPP_INFO(this->get_logger(), "Telemetry component successfully created.");

	// ROS 2 related - Publisher callbacks
	antenna_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
	                              "antenna_pose", queue_size_);

	antenna_pose_timer_ = this->create_wall_timer(
	                          antenna_pose_period_ms_, std::bind(&TelemetryComponent::AntennaPoseCallback,
	                                  this));

	// ROS 2 related - Subscriber callbacks
	pulse_subscriber_ = this->create_subscription<uavrt_interfaces::msg::Pulse>(
	                        "pulse", queue_size_, std::bind(&TelemetryComponent::PulseCallback,
	                                this,
	                                std::placeholders::_1));

	pulse_pose_publisher_ = this->create_publisher<uavrt_interfaces::msg::PulsePose>(
	                            "pulse_pose", queue_size_);

	// MAVSDK related - Subsriber rate variables
	mavsdk_telemetry.set_rate_position(postion_subscribe_rate_);
	mavsdk_telemetry.set_rate_attitude(attitude_subscribe_rate_);

	// MAVSDK related - Subscriber callbacks
	system->subscribe_is_connected(std::bind(&TelemetryComponent::ConnectionCallback,
	                               this,
	                               std::placeholders::_1));

	mavsdk_telemetry.subscribe_position(std::bind(&TelemetryComponent::PositionCallback,
	                                    this,
	                                    std::placeholders::_1));

	mavsdk_telemetry.subscribe_attitude_quaternion(std::bind(&TelemetryComponent::QuaternionCallback,
	        this,
	        std::placeholders::_1));
}

// I tried placing this function in main and passing a shared pointer to each
// of the components in the hopes of reducing code redundancy (i.e. this
// connection will need to be done in all components seperatly). However,
// I was unable to set the shared pointer within main to a different pointer.
// I rather ended up only changing the contents of the pointer within the scope
// of main or reset the contents of the pointer but the references within
// the components would fail.
// Ideally, we would use a publisher/subscriber, but you can't create a publisher
// within main since it is not a node (i.e. doesn't inherit from the node class).
//
// Search for the following terms in the link below for more info on "is_connected":
// typedef IsConnectedCallback, is_connected(), subscribe_is_connected()
// https://mavsdk.mavlink.io/main/en/cpp/api_reference/classmavsdk_1_1_system.html
void TelemetryComponent::ConnectionCallback(bool is_connected)
{
	connection_status_ = is_connected;
	RCLCPP_ERROR(this->get_logger(),
	             "MAVSDK system connection has timed out! The UAV-RT system is "
	             "no longer collecting data from the Pixhawk autopilot!");
}

void TelemetryComponent::AntennaPoseCallback()
{
	if (connection_status_ == true)
	{
		// https://docs.ros2.org/galactic/api/std_msgs/msg/Header.html
		antenna_pose_header_ = std_msgs::msg::Header();
		// https://docs.ros2.org/galactic/api/geometry_msgs/msg/PoseStamped.html
		antenna_pose_pose_stamped_ = geometry_msgs::msg::PoseStamped();
		// https://docs.ros2.org/galactic/api/geometry_msgs/msg/Pose.html
		antenna_pose_pose_ = geometry_msgs::msg::Pose();
		// https://docs.ros2.org/galactic/api/geometry_msgs/msg/Point.html
		antenna_pose_point_ = geometry_msgs::msg::Point();
		// https://docs.ros2.org/galactic/api/geometry_msgs/msg/Quaternion.html
		antenna_pose_quaternion_ = geometry_msgs::msg::Quaternion();

		// https://docs.ros2.org/galactic/api/builtin_interfaces/msg/Time.html
		antenna_pose_header_.stamp = this->get_clock()->now();
		antenna_pose_header_.frame_id = "antenna_pose";

		std::string antenna_pose_time_value_ =
		    std::to_string(antenna_pose_header_.stamp.sec) + "." +
		    std::to_string(antenna_pose_header_.stamp.nanosec);
		antenna_pose_time_vector_.push_back(std::stod(antenna_pose_time_value_));

		antenna_pose_point_.x = position_.latitude_deg;
		antenna_pose_point_.y = position_.longitude_deg;
		antenna_pose_point_.z = position_.absolute_altitude_m;

		// This seems reduntant to store the position values above and then
		// store them into their respective arrays. But if we stored the values
		// in the position vectors and then accessed them for assignment
		// afterwards, we would spend time transversing the array and vectors
		// for the position values. Same for quaternion.
		antenna_pose_position_array_[
		    static_cast<int>(PositionArray::LatitudeIndex)].push_back(antenna_pose_point_.x);
		antenna_pose_position_array_[
		    static_cast<int>(PositionArray::LongitudeIndex)].push_back(antenna_pose_point_.y);
		antenna_pose_position_array_[
		    static_cast<int>(PositionArray::AltitudeIndex)].push_back(antenna_pose_point_.z);

		antenna_pose_quaternion_.w = quaternion_.w;
		antenna_pose_quaternion_.x = quaternion_.x;
		antenna_pose_quaternion_.y = quaternion_.y;
		antenna_pose_quaternion_.z = quaternion_.z;

		antenna_pose_quaternion_array_[
		    static_cast<int>(QuaternionArray::WIndex)].push_back(antenna_pose_quaternion_.w);
		antenna_pose_quaternion_array_[
		    static_cast<int>(QuaternionArray::XIndex)].push_back(antenna_pose_quaternion_.x);
		antenna_pose_quaternion_array_[
		    static_cast<int>(QuaternionArray::YIndex)].push_back(antenna_pose_quaternion_.y);
		antenna_pose_quaternion_array_[
		    static_cast<int>(QuaternionArray::ZIndex)].push_back(antenna_pose_quaternion_.z);

		antenna_pose_pose_.position = antenna_pose_point_;
		antenna_pose_pose_.orientation = antenna_pose_quaternion_;

		antenna_pose_pose_stamped_.header = antenna_pose_header_;
		antenna_pose_pose_stamped_.pose = antenna_pose_pose_;

		antenna_pose_publisher_->publish(antenna_pose_pose_stamped_);

		RCLCPP_INFO(this->get_logger(),
		            "Successfully published /antenna_pose.");
	}
}

// I'm not sure how to detect if there was an error getting the position
// attributes from the Telemetry object. E.g. the Pixhawk is unplugged, then
// the statements below should report that there was an issue.
// MAVSDK doesn't use exceptions but instead enum values. But I'm not sure
// what I'm supposed to be comparing to the enum value for success or failure.
// There isn't a result associated with the statements below.
// https://mavsdk.mavlink.io/main/en/cpp/guide/general_usage.html#error-handling
void TelemetryComponent::PositionCallback(mavsdk::Telemetry::Position position)
{
	// Poll for 'Position' (blocking).
	// https://mavsdk.mavlink.io/main/en/cpp/api_reference/classmavsdk_1_1_telemetry.html#classmavsdk_1_1_telemetry_1a61bda57b3ca47000ea7e4758b2a33134
	this->position_ = position;
}

void TelemetryComponent::QuaternionCallback(mavsdk::Telemetry::Quaternion quaternion)
{
	// Subscribe to 'attitude' updates (quaternion).
	// https://mavsdk.mavlink.io/main/en/cpp/api_reference/classmavsdk_1_1_telemetry.html#classmavsdk_1_1_telemetry_1afa6c079d48bc0c0a3287ac095ec290b9
	this->quaternion_ = quaternion;
}

void TelemetryComponent::PulseCallback(
    uavrt_interfaces::msg::Pulse::SharedPtr pulse_message)
{
	RCLCPP_INFO(this->get_logger(),
	            "Successfully received pulse.");

	if (connection_status_ == true)
	{
		InterpolationResults retrieved_results;

		// https://docs.ros2.org/galactic/api/geometry_msgs/msg/Pose.html
		pulse_pose_ = geometry_msgs::msg::Pose();
		// https://docs.ros2.org/galactic/api/geometry_msgs/msg/Point.html
		pulse_point_ = geometry_msgs::msg::Point();
		// https://docs.ros2.org/galactic/api/geometry_msgs/msg/Quaternion.html
		pulse_quaternion_ = geometry_msgs::msg::Quaternion();
		// https://github.com/dynamic-and-active-systems-lab/uavrt_interfaces/blob/main/msg/Pulse.msg
		pulse_pulse_pose_ = uavrt_interfaces::msg::PulsePose();

		std::string pulse_start_time =
		    std::to_string(pulse_message->start_time.sec) + "." +
		    std::to_string(pulse_message->start_time.nanosec);
		std::string pulse_end_time =
		    std::to_string(pulse_message->end_time.sec) + "." +
		    std::to_string(pulse_message->end_time.nanosec);
		double pulse_average_time =
		    (std::stod(pulse_start_time) + std::stod(pulse_end_time)) / 2;

		// Interpolate/Slerp
		retrieved_results = InterpolatePosition(pulse_average_time);

		// Position
		pulse_point_.x = retrieved_results.interpolated_latitude;
		pulse_point_.y = retrieved_results.interpolated_logitude;
		pulse_point_.z = retrieved_results.interpolated_altitude;

		// Quaternion
		pulse_quaternion_.w =
		    retrieved_results.interpolated_quaternion.a[static_cast<int>(QuaternionArray::WIndex)];
		pulse_quaternion_.x =
		    retrieved_results.interpolated_quaternion.a[static_cast<int>(QuaternionArray::XIndex)];
		pulse_quaternion_.y =
		    retrieved_results.interpolated_quaternion.a[static_cast<int>(QuaternionArray::YIndex)];
		pulse_quaternion_.z =
		    retrieved_results.interpolated_quaternion.a[static_cast<int>(QuaternionArray::ZIndex)];

		// Create PulsePose object
		pulse_pose_.position = pulse_point_;
		pulse_pose_.orientation = pulse_quaternion_;

		pulse_pulse_pose_.pulse = *pulse_message;
		pulse_pulse_pose_.antenna_pose = pulse_pose_;

		// Use CommandComponent object to send to the ground
		pulse_pose_publisher_->publish(pulse_pulse_pose_);

		RCLCPP_INFO(this->get_logger(),
		            "Successfully published pulse_pose.");

		LogPulsePose(pulse_pulse_pose_);
	}
	else if (connection_status_ == false)
	{
		RCLCPP_ERROR(this->get_logger(),
		             "Unable to retrieve telemetry data! This is most likely "
		             "because the MAVSDK system connection has timed out.");
	}
}

// Help withing find functionality:
// https://stackoverflow.com/questions/44179512/find-the-first-element-strictly-less-than-a-key-in-a-vector-sorted-in-descending
// std::begin/std::end vs .end() and .begin:
// https://stackoverflow.com/questions/8452130/when-to-use-stdbegin-and-stdend-instead-of-container-specific-versions
InterpolationResults TelemetryComponent::InterpolatePosition(
    double pulse_average_time)
{
	InterpolationResults generated_results;
	uint16_t seven_hours_in_seconds = 25200;

	// The time we are receiving from the detectors is recorded using GMT -7.
	// We add seven hours to the average times in order to bring them to GMT 0.
	// This is a bandaid. All uavrt packages should be using the same timezone.
	pulse_average_time = pulse_average_time + seven_hours_in_seconds;

	// To check contents of vectors:
	// #include <iostream>     // std::fixed
	// #include <iomanip>      // std::setprecision
	// std::cout << std::fixed;
	// for (auto it = antenna_pose_time_vector_.begin();
	//      it != antenna_pose_time_vector_.end(); it++) {
	//     std::cout << std::setprecision(9) << "antenna_pose_time_vector_: " << *it << std::endl;
	// }

	// This will return the first value that is less than the key
	std::vector<double>::iterator upper_bound_iterator = std::upper_bound(
	            antenna_pose_time_vector_.begin(),
	            antenna_pose_time_vector_.end(),
	            pulse_average_time,
	            std::greater<double>());

	// This will return the first value that is greater than the key
	std::vector<double>::iterator lower_bound_iterator = std::lower_bound(
	            antenna_pose_time_vector_.begin(),
	            antenna_pose_time_vector_.end(),
	            pulse_average_time,
	            std::less<double>());

	if (upper_bound_iterator == antenna_pose_time_vector_.end() &&
	        lower_bound_iterator == antenna_pose_time_vector_.end())
	{
		RCLCPP_ERROR(this->get_logger(),
		             "There does not exist a stored time value that is lower "
		             "and/or greater than the pulse average time that was provided.");

		generated_results.interpolated_latitude = 0.0;
		generated_results.interpolated_logitude = 0.0;
		generated_results.interpolated_altitude = 0.0;

		generated_results.interpolated_quaternion.a[static_cast<int>(QuaternionArray::WIndex)] = 0.0;
		generated_results.interpolated_quaternion.a[static_cast<int>(QuaternionArray::XIndex)] = 0.0;
		generated_results.interpolated_quaternion.a[static_cast<int>(QuaternionArray::YIndex)] = 0.0;
		generated_results.interpolated_quaternion.a[static_cast<int>(QuaternionArray::ZIndex)] = 0.0;

		return generated_results;
	}

	RCLCPP_INFO(this->get_logger(),
	            "Pulse average time was found in the stored time values.");

	// Help with find function:
	// https://stackoverflow.com/questions/571394/how-to-find-out-if-an-item-is-present-in-a-stdvector
	// https://stackoverflow.com/questions/15099707/how-to-get-position-of-a-certain-element-in-strings-vector-to-use-it-as-an-inde
	// I don't check whether value is valid at this point, since we would
	// have been stopped early if the value didn't exist.
	int upper_bound_index =
	    std::distance(antenna_pose_time_vector_.begin(),
	                  std::find(antenna_pose_time_vector_.begin(),
	                            antenna_pose_time_vector_.end(),
	                            *upper_bound_iterator));
	int lower_bound_index =
	    std::distance(antenna_pose_time_vector_.begin(),
	                  std::find(antenna_pose_time_vector_.begin(),
	                            antenna_pose_time_vector_.end(),
	                            *lower_bound_iterator));

	if (upper_bound_index > 0) {upper_bound_index -= 1;}
	else if (lower_bound_index > 0) {lower_bound_index -= 1;}

	generated_results.interpolated_latitude = std::lerp(
	            antenna_pose_position_array_[static_cast<int>(PositionArray::LatitudeIndex)].at(upper_bound_index),
	            antenna_pose_position_array_[static_cast<int>(PositionArray::LatitudeIndex)].at(lower_bound_index),
	            0.5);

	generated_results.interpolated_logitude = std::lerp(
	            antenna_pose_position_array_[static_cast<int>(PositionArray::LongitudeIndex)].at(upper_bound_index),
	            antenna_pose_position_array_[static_cast<int>(PositionArray::LongitudeIndex)].at(lower_bound_index),
	            0.5);

	generated_results.interpolated_altitude = std::lerp(
	            antenna_pose_position_array_[static_cast<int>(PositionArray::AltitudeIndex)].at(upper_bound_index),
	            antenna_pose_position_array_[static_cast<int>(PositionArray::AltitudeIndex)].at(lower_bound_index),
	            0.5);

	// Quaternion interpolation between two quaternions
	// We use the boost::qvm::slerp function for this:
	// https://www.boost.org/doc/libs/1_71_0/libs/qvm/doc/html/index.html#slerp
	// https://github.com/boostorg/qvm/blob/5791440b346232c391ab8d16f559ca5b2d7ae9b3/include/boost/qvm/quat_operations.hpp#L717
	// You need to create boost::qvm::quat(ernion) structs in order to use the
	// slerp function.
	// https://github.com/boostorg/qvm/blob/5791440b346232c391ab8d16f559ca5b2d7ae9b3/include/boost/qvm/quat.hpp#L15
	// You can then access the quaternion structs and edit their member variables.
	//
	// Note: boost::qvm::slerp will not normalize the data for you!
	// The data provided by MAVSDK is already normalized. In the event you
	// need to normalize with boost, the syntax is this:
	// boost::qvm::normalize(quaternion);
	//
	// Thank you to this SO post for making it easier to understand how I'm
	// supposed to use these pieces of Boost functionality.
	// https://stackoverflow.com/questions/70385110/boost-qvms-normalized-for-quaternions-doesnt-work-with-boost-multiprecision
    // https://www.boost.org/doc/libs/1_80_0/libs/qvm/doc/html/index.html
	boost::qvm::quat<float> pulse_quaternion_upper_index;

	pulse_quaternion_upper_index.a[static_cast<int>(QuaternionArray::WIndex)] =
	    antenna_pose_quaternion_array_[static_cast<int>(QuaternionArray::WIndex)].at(upper_bound_index);
	pulse_quaternion_upper_index.a[static_cast<int>(QuaternionArray::XIndex)] =
	    antenna_pose_quaternion_array_[static_cast<int>(QuaternionArray::XIndex)].at(upper_bound_index);
	pulse_quaternion_upper_index.a[static_cast<int>(QuaternionArray::YIndex)] =
	    antenna_pose_quaternion_array_[static_cast<int>(QuaternionArray::YIndex)].at(upper_bound_index);
	pulse_quaternion_upper_index.a[static_cast<int>(QuaternionArray::ZIndex)] =
	    antenna_pose_quaternion_array_[static_cast<int>(QuaternionArray::ZIndex)].at(upper_bound_index);

	boost::qvm::quat<float> pulse_quaternion_lower_index;

	pulse_quaternion_lower_index.a[static_cast<int>(QuaternionArray::WIndex)] =
	    antenna_pose_quaternion_array_[static_cast<int>(QuaternionArray::WIndex)].at(lower_bound_index);
	pulse_quaternion_lower_index.a[static_cast<int>(QuaternionArray::XIndex)] =
	    antenna_pose_quaternion_array_[static_cast<int>(QuaternionArray::XIndex)].at(lower_bound_index);
	pulse_quaternion_lower_index.a[static_cast<int>(QuaternionArray::YIndex)] =
	    antenna_pose_quaternion_array_[static_cast<int>(QuaternionArray::YIndex)].at(lower_bound_index);
	pulse_quaternion_lower_index.a[static_cast<int>(QuaternionArray::ZIndex)] =
	    antenna_pose_quaternion_array_[static_cast<int>(QuaternionArray::ZIndex)].at(lower_bound_index);

	boost::qvm::quat<float> pulse_quaternion_slerp_result =
	    boost::qvm::slerp(
	        pulse_quaternion_upper_index,
	        pulse_quaternion_lower_index,
	        0.5);

	generated_results.interpolated_quaternion = pulse_quaternion_slerp_result;

	return generated_results;
}

void TelemetryComponent::LogPulsePose(uavrt_interfaces::msg::PulsePose
                                      pulse_pose_message)
{
	// Dynamic pulse_pose_file_location (directory info from Pulse msg)
	std::string pulse_pose_log_file_location =
	    (pulse_pose_message.pulse.detector_dir).append("/output/pulse_pose_log.txt");

	std::ofstream pulse_pose_log_file;

	// Open for output in append mode (create a new file only if the file does not exist)
	// https://cplusplus.com/doc/tutorial/files/
	pulse_pose_log_file.open(pulse_pose_log_file_location,
	                         std::ios::app);
	if (pulse_pose_log_file.is_open() == true)
	{
		pulse_pose_log_file << pulse_pose_message.pulse.tag_id << ", ";
		pulse_pose_log_file << pulse_pose_message.pulse.frequency << ", ";
		pulse_pose_log_file << pulse_pose_message.pulse.start_time.sec << ", ";
		pulse_pose_log_file << pulse_pose_message.pulse.start_time.nanosec << ", ";
		pulse_pose_log_file << pulse_pose_message.pulse.end_time.sec << ", ";
		pulse_pose_log_file << pulse_pose_message.pulse.end_time.nanosec << ", ";
		pulse_pose_log_file << pulse_pose_message.pulse.predict_next_start.sec << ", ";
		pulse_pose_log_file << pulse_pose_message.pulse.predict_next_start.nanosec << ", ";
		pulse_pose_log_file << pulse_pose_message.pulse.predict_next_end.sec << ", ";
		pulse_pose_log_file << pulse_pose_message.pulse.predict_next_end.nanosec << ", ";
		pulse_pose_log_file << pulse_pose_message.pulse.snr << ", ";
		pulse_pose_log_file << pulse_pose_message.pulse.stft_score << ", ";
		pulse_pose_log_file << pulse_pose_message.pulse.group_ind << ", ";
		pulse_pose_log_file << pulse_pose_message.pulse.group_snr << ", ";
		pulse_pose_log_file << pulse_pose_message.pulse.detection_status << ", ";
		pulse_pose_log_file << pulse_pose_message.pulse.confirmed_status << ", ";

		pulse_pose_log_file << pulse_pose_message.antenna_pose.position.x << ", ";
		pulse_pose_log_file << pulse_pose_message.antenna_pose.position.y << ", ";
		pulse_pose_log_file << pulse_pose_message.antenna_pose.position.z << ", ";

		pulse_pose_log_file << pulse_pose_message.antenna_pose.orientation.x << ", ";
		pulse_pose_log_file << pulse_pose_message.antenna_pose.orientation.y << ", ";
		pulse_pose_log_file << pulse_pose_message.antenna_pose.orientation.z << ", ";
		pulse_pose_log_file << pulse_pose_message.antenna_pose.orientation.w << "\n";
	}
	else if (pulse_pose_log_file.is_open() == false)
	{
		RCLCPP_ERROR(this->get_logger(),
		             "Unable to open/append to pulse pose log file!");
	}

	pulse_pose_log_file.close();
}

} // namespace uavrt_connection
