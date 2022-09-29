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

	stop_command_diagnostic_array_.status[
		static_cast<int>(DiagnosticStatusIndices::DiagnosticStatus)] =
		      stop_command_diagnostic_status_;

	stop_command_diagnostic_array_.header = stop_command_header_;

	RCLCPP_INFO(this->get_logger(),
	            "Successfully received stop command.");

	stop_subprocesses_publisher_->publish(stop_command_diagnostic_array_);

	HandleAckCommand(static_cast<int>(uavrt_interfaces::CommandID::CommandIDTag),
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

	// NOTE: We currently not using all of the parameters within the ROS 2
	// pulse_pose messages. Certain parameters are also hardcoded for the time
	// being.
	outgoing_debug_float_array.array_id = static_cast<int>(uavrt_interfaces::CommandID::CommandIDPulse);
	outgoing_debug_float_array.data[static_cast<int>(uavrt_interfaces::PulseIndex::PulseIndexDetectionStatus)] = 2;
	outgoing_debug_float_array.data[
		static_cast<int>(uavrt_interfaces::PulseIndex::PulseIndexStrength)] =
		pulse_pose_message->pulse.snr;
	outgoing_debug_float_array.data[
		static_cast<int>(uavrt_interfaces::PulseIndex::PulseIndexGroupIndex)] =
		pulse_pose_message->pulse.group_ind;

	mavlink_msg_debug_float_array_encode(
		mavlink_passthrough_.get_our_sysid(),
		mavlink_passthrough_.get_our_compid(),
		&message,
		&outgoing_debug_float_array);

	mavlink_passthrough_.send_message(message);

	RCLCPP_INFO(this->get_logger(),
	            "Successfully sent pulse pose message to the ground.");
}

} // namespace uavrt_connection
