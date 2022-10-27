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

// C++ standard library headers
#include <functional>
#include <memory>
#include <string>

// Delete - only for debugging
#include <cmath>
#include <iostream>     // std::cout, std::fixed
#include <iomanip>      // std::setprecision

// Project header files
#include "uavrt_connection/command_component.hpp"

namespace uavrt_connection
{

CommandComponent::CommandComponent(const rclcpp::NodeOptions& options,
                                   std::shared_ptr<mavsdk::System> system)
	: Node("CommandComponent", options),
	mavlink_passthrough_(system)
{
	RCLCPP_INFO(this->get_logger(), "Command component successfully created.");

	// ROS 2 related - Publisher callbacks
	start_subprocesses_publisher_ = this->create_publisher<uavrt_interfaces::msg::TagDef>(
		"control_start_subprocess",
		queue_size_);

	// ROS 2 related - Publisher callbacks
	stop_subprocesses_publisher_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
		"control_stop_subprocess",
		queue_size_);

	// ROS 2 related - Subscriber callbacks
	pulse_pose_subscriber_ = this->create_subscription<uavrt_interfaces::msg::PulsePose>(
		"pulse_pose", queue_size_, std::bind(&CommandComponent::HandlePulseCommand,
		                                     this,
		                                     std::placeholders::_1));

	// MAVSDK related
	mavlink_passthrough_.intercept_incoming_messages_async(std::bind(&CommandComponent::CommandCallback,
	                                                                 this,
	                                                                 std::placeholders::_1));
}

// Static_cast is required for enum classes. Enum classes are safer than
// enum since it's not an implicit conversion to a data type like int.
// https://rules.sonarsource.com/cpp/RSPEC-3642
bool CommandComponent::CommandCallback(mavlink_message_t& message)
{
	if (message.msgid == MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY)
	{
		mavlink_debug_float_array_t debugFloatArray;

		mavlink_msg_debug_float_array_decode(&message, &debugFloatArray);

		switch (debugFloatArray.array_id)
		{
		case static_cast<int>(uavrt_interfaces::CommandID::CommandIDStart):
			HandleStartCommand();
			break;
		case static_cast<int>(uavrt_interfaces::CommandID::CommandIDStop):
			HandleStopCommand();
			break;
		case static_cast<int>(uavrt_interfaces::CommandID::CommandIDTag):
			HandleTagCommand(debugFloatArray);
			break;
		}
	}

	// To drop a message, return 'false' from the callback.
	return true;
}

void CommandComponent::HandleAckCommand(uint32_t command_id, uint32_t result)
{
	mavlink_message_t message;
	mavlink_debug_float_array_t outgoing_debug_float_array;

	memset(&outgoing_debug_float_array, 0, sizeof(outgoing_debug_float_array));

	outgoing_debug_float_array.array_id = static_cast<int>(uavrt_interfaces::CommandID::CommandIDAck);
	outgoing_debug_float_array.data[static_cast<int>(uavrt_interfaces::AckIndex::AckIndexCommand)] = command_id;
	outgoing_debug_float_array.data[static_cast<int>(uavrt_interfaces::AckIndex::AckIndexResult)]  = result;

	mavlink_msg_debug_float_array_encode(
		mavlink_passthrough_.get_our_sysid(),
		mavlink_passthrough_.get_our_compid(),
		&message,
		&outgoing_debug_float_array);

	mavlink_passthrough_.send_message(message);
}

void CommandComponent::HandleStartCommand()
{
	// This Command is currently not utilized. We send back a positive ack,
	// else QGC will not allow for takeoff.

	uint32_t command_result = 1;

	RCLCPP_INFO(this->get_logger(),
	            "Successfully received start command.");

	HandleAckCommand(static_cast<int>(uavrt_interfaces::CommandID::CommandIDStart),
	                 command_result);
}

void CommandComponent::HandleStopCommand()
{
	uint32_t command_result = 1;

	// https://docs.ros2.org/galactic/api/std_msgs/msg/Header.html
	stop_command_header_ = std_msgs::msg::Header();
	// https://docs.ros2.org/galactic/api/diagnostic_msgs/msg/DiagnosticArray.html
	stop_command_diagnostic_array_ = diagnostic_msgs::msg::DiagnosticArray();
	// https://docs.ros2.org/galactic/api/diagnostic_msgs/msg/DiagnosticStatus.html
	stop_command_diagnostic_status_ = diagnostic_msgs::msg::DiagnosticStatus();
	// https://docs.ros2.org/galactic/api/diagnostic_msgs/msg/KeyValue.html
	stop_command_key_value_ = diagnostic_msgs::msg::KeyValue();

	// https://docs.ros2.org/galactic/api/builtin_interfaces/msg/Time.html
	stop_command_header_.stamp = this->get_clock()->now();
	stop_command_header_.frame_id = "stop_command_subprocesses";

	stop_command_diagnostic_status_.level = '0';
	stop_command_diagnostic_status_.name = "NA";
	stop_command_diagnostic_status_.message = "stop";
	stop_command_diagnostic_status_.hardware_id = "stop all";

	stop_command_diagnostic_array_.status.push_back(stop_command_diagnostic_status_);

	stop_command_diagnostic_array_.header = stop_command_header_;

	RCLCPP_INFO(this->get_logger(),
	            "Successfully received stop command.");

	stop_subprocesses_publisher_->publish(stop_command_diagnostic_array_);

	HandleAckCommand(static_cast<int>(uavrt_interfaces::CommandID::CommandIDStop),
	                 command_result);
}

void CommandComponent::HandleTagCommand(const mavlink_debug_float_array_t& debug_float_array)
{
	uint32_t command_result = 1;

	// https://github.com/dynamic-and-active-systems-lab/uavrt_interfaces/blob/main/msg/TagDef.msg
	tag_info_ = uavrt_interfaces::msg::TagDef();

	tag_info_.tag_id = debug_float_array.data[static_cast<int>(uavrt_interfaces::TagIndex::TagIndexID)];
	tag_info_.frequency = debug_float_array.data[static_cast<int>(uavrt_interfaces::TagIndex::TagIndexFrequency)];
	tag_info_.pulse_duration = debug_float_array.data[static_cast<int>(uavrt_interfaces::TagIndex::TagIndexDurationMSecs)];
	tag_info_.interpulse_time_1 = debug_float_array.data[static_cast<int>(uavrt_interfaces::TagIndex::TagIndexIntraPulse1MSecs)];
	tag_info_.interpulse_time_2 = debug_float_array.data[static_cast<int>(uavrt_interfaces::TagIndex::TagIndexIntraPulse2MSecs)];
	tag_info_.interpulse_time_uncert = debug_float_array.data[static_cast<int>(uavrt_interfaces::TagIndex::TagIndexIntraPulseUncertainty)];
	tag_info_.interpulse_time_jitter = debug_float_array.data[static_cast<int>(uavrt_interfaces::TagIndex::TagIndexIntraPulseJitter)];
	// ?? _simulatorMaxPulse = debug_float_array.data[static_cast<int>(TagIndex::TagIndexMaxPulse)];

	if (tag_info_.tag_id[0] == 0)
	{
		RCLCPP_ERROR(this->get_logger(), "Invalid tag id of 0.");
		command_result  = 0;
	}
	else if (tag_info_.tag_id[0] != 0)
	{
		RCLCPP_INFO(this->get_logger(),
		            "Successfully received tag info.");

		start_subprocesses_publisher_->publish(tag_info_);
	}

	HandleAckCommand(static_cast<int>(uavrt_interfaces::CommandID::CommandIDTag),
	                 command_result);
}

void CommandComponent::HandlePulseCommand(
	uavrt_interfaces::msg::PulsePose::SharedPtr pulse_pose_message)
{
	mavlink_message_t message;
	mavlink_debug_float_array_t outgoing_debug_float_array;

	memset(&outgoing_debug_float_array, 0, sizeof(outgoing_debug_float_array));

	outgoing_debug_float_array.array_id = static_cast<int>(uavrt_interfaces::CommandID::CommandIDPulse);

	// The first three entries (0-2) are kept here for the purpose of legacy
	// support with QGC. They should be removed in the future to reduce redundancy.
	outgoing_debug_float_array.data[
		static_cast<int>(uavrt_interfaces::PulseIndex::PulseIndexDetectionStatusLEGACY)] = 0;
	outgoing_debug_float_array.data[
		static_cast<int>(uavrt_interfaces::PulseIndex::PulseIndexStrengthLEGACY)] =
		pulse_pose_message->pulse.snr;
	outgoing_debug_float_array.data[
		static_cast<int>(uavrt_interfaces::PulseIndex::PulseIndexGroupIndexLEGACY)] = 0;

	// Refer to uavrt_interfaces/include/uavrt_interfaces/qgc_enum_class_definitions.hpp
	// for descriptions on each of the data values.
	outgoing_debug_float_array.data[
		static_cast<int>(uavrt_interfaces::PulseIndex::PulseIndexTagID)] =
		std::stof(pulse_pose_message->pulse.detector_id);
	outgoing_debug_float_array.data[
		static_cast<int>(uavrt_interfaces::PulseIndex::PulseIndexFrequency)] =
		pulse_pose_message->pulse.frequency;
	outgoing_debug_float_array.data[
		static_cast<int>(uavrt_interfaces::PulseIndex::PulseIndexStartTime)] =
		TimeToFloat(pulse_pose_message->pulse.start_time.sec,
		            pulse_pose_message->pulse.start_time.nanosec);
	outgoing_debug_float_array.data[
		static_cast<int>(uavrt_interfaces::PulseIndex::PulseIndexEndTime)] =
		TimeToFloat(pulse_pose_message->pulse.end_time.sec,
		            pulse_pose_message->pulse.end_time.nanosec);
	outgoing_debug_float_array.data[
		static_cast<int>(uavrt_interfaces::PulseIndex::PulseIndexPredictNextStartTime)] =
		TimeToFloat(pulse_pose_message->pulse.predict_next_start.sec,
		            pulse_pose_message->pulse.predict_next_start.nanosec);
	outgoing_debug_float_array.data[
		static_cast<int>(uavrt_interfaces::PulseIndex::PulseIndexPredictNextEndTime)] =
		TimeToFloat(pulse_pose_message->pulse.predict_next_end.sec,
		            pulse_pose_message->pulse.predict_next_end.nanosec);
	outgoing_debug_float_array.data[
		static_cast<int>(uavrt_interfaces::PulseIndex::PulseIndexSNR)] =
		pulse_pose_message->pulse.snr;
	outgoing_debug_float_array.data[
		static_cast<int>(uavrt_interfaces::PulseIndex::PulseIndexSNRPerSample)] =
		pulse_pose_message->pulse.snr_per_sample;
	outgoing_debug_float_array.data[
		static_cast<int>(uavrt_interfaces::PulseIndex::PulseIndexPSDSignalNoise)] =
		pulse_pose_message->pulse.psd_sn;
	outgoing_debug_float_array.data[
		static_cast<int>(uavrt_interfaces::PulseIndex::PulseIndexPSDNoise)] =
		pulse_pose_message->pulse.psd_n;
	outgoing_debug_float_array.data[
		static_cast<int>(uavrt_interfaces::PulseIndex::PulseIndexDFTReal)] =
		pulse_pose_message->pulse.dft_real;
	outgoing_debug_float_array.data[
		static_cast<int>(uavrt_interfaces::PulseIndex::PulseIndexDFTImaginary)] =
		pulse_pose_message->pulse.dft_imag;
	outgoing_debug_float_array.data[
		static_cast<int>(uavrt_interfaces::PulseIndex::PulseIndexGroupIndex)] =
		pulse_pose_message->pulse.group_ind;
	outgoing_debug_float_array.data[
		static_cast<int>(uavrt_interfaces::PulseIndex::PulseIndexGroupSNR)] =
		pulse_pose_message->pulse.group_snr;
	outgoing_debug_float_array.data[
		static_cast<int>(uavrt_interfaces::PulseIndex::PulseIndexDetectionStatus)] =
		pulse_pose_message->pulse.detection_status;
	outgoing_debug_float_array.data[
		static_cast<int>(uavrt_interfaces::PulseIndex::PulseIndexConfirmedStatus)] =
		pulse_pose_message->pulse.confirmed_status;
	outgoing_debug_float_array.data[
		static_cast<int>(uavrt_interfaces::PulseIndex::PulseIndexPositionLongitude)] =
		pulse_pose_message->antenna_pose.position.x;
	outgoing_debug_float_array.data[
		static_cast<int>(uavrt_interfaces::PulseIndex::PulseIndexPositionLatitude)] =
		pulse_pose_message->antenna_pose.position.y;
	outgoing_debug_float_array.data[
		static_cast<int>(uavrt_interfaces::PulseIndex::PulseIndexPositionAltitude)] =
		pulse_pose_message->antenna_pose.position.z;
	outgoing_debug_float_array.data[
		static_cast<int>(uavrt_interfaces::PulseIndex::PulseIndexQuaternionX)] =
		pulse_pose_message->antenna_pose.orientation.x;
	outgoing_debug_float_array.data[
		static_cast<int>(uavrt_interfaces::PulseIndex::PulseIndexQuaternionY)] =
		pulse_pose_message->antenna_pose.orientation.y;
	outgoing_debug_float_array.data[
		static_cast<int>(uavrt_interfaces::PulseIndex::PulseIndexQuaternionZ)] =
		pulse_pose_message->antenna_pose.orientation.z;
	outgoing_debug_float_array.data[
		static_cast<int>(uavrt_interfaces::PulseIndex::PulseIndexQuaternionW)] =
		pulse_pose_message->antenna_pose.orientation.w;

	mavlink_msg_debug_float_array_encode(
		mavlink_passthrough_.get_our_sysid(),
		mavlink_passthrough_.get_our_compid(),
		&message,
		&outgoing_debug_float_array);

	mavlink_passthrough_.send_message(message);

	RCLCPP_INFO(this->get_logger(),
	            "Successfully sent pulse pose message to the ground.");
}

// Helper function for values that can not be inherently converted to floats
// (like string and ROS 2 builtin_interfaces/Time). These values need to be
// manually converted.
float CommandComponent::TimeToFloat(int seconds, uint32_t nanoseconds)
{
	std::string uncoverted_time_value =
		std::to_string(seconds) + "." +
		std::to_string(nanoseconds);

	float converted_time_value = std::stof(uncoverted_time_value);

	return converted_time_value;
}

} // namespace uavrt_connection
