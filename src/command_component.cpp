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
	tag_publisher_ = this->create_publisher<uavrt_interfaces::msg::TagDef>(
		"/tag_info", queue_size_);

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
		case static_cast<int>(CommandID::CommandIDTag):
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

    // Better way to do this over using memset?
	memset(&outgoing_debug_float_array, 0, sizeof(outgoing_debug_float_array));

	outgoing_debug_float_array.array_id = static_cast<int>(CommandID::CommandIDAck);
	outgoing_debug_float_array.data[static_cast<int>(AckIndex::AckIndexCommand)] = command_id;
	outgoing_debug_float_array.data[static_cast<int>(AckIndex::AckIndexResult)]  = result;

	mavlink_msg_debug_float_array_encode(
		mavlink_passthrough_.get_our_sysid(),
		mavlink_passthrough_.get_our_compid(),
		&message,
		&outgoing_debug_float_array);

	mavlink_passthrough_.send_message(message);
}

void CommandComponent::HandleTagCommand(const mavlink_debug_float_array_t& debug_float_array)
{
	uint32_t command_result = 1;

	// https://github.com/dynamic-and-active-systems-lab/uavrt_interfaces/blob/main/msg/TagDef.msg
	tag_info_ = uavrt_interfaces::msg::TagDef();

	tag_info_.tag_id = debug_float_array.data[static_cast<int>(TagIndex::TagIndexID)];
	tag_info_.frequency = debug_float_array.data[static_cast<int>(TagIndex::TagIndexFrequency)];
	tag_info_.pulse_duration = debug_float_array.data[static_cast<int>(TagIndex::TagIndexDurationMSecs)];
	tag_info_.interpulse_time_1 = debug_float_array.data[static_cast<int>(TagIndex::TagIndexIntraPulse1MSecs)];
	tag_info_.interpulse_time_2 = debug_float_array.data[static_cast<int>(TagIndex::TagIndexIntraPulse2MSecs)];
	tag_info_.interpulse_time_uncert = debug_float_array.data[static_cast<int>(TagIndex::TagIndexIntraPulseUncertainty)];
	tag_info_.interpulse_time_jitter = debug_float_array.data[static_cast<int>(TagIndex::TagIndexIntraPulseJitter)];
	// ?? _simulatorMaxPulse = debug_float_array.data[static_cast<int>(TagIndex::TagIndexMaxPulse)];

	if (tag_info_.tag_id[0] == 0)
	{
		RCLCPP_ERROR(this->get_logger(), "Invalid tag id of 0.");
		command_result  = 0;
	}

	HandleAckCommand(static_cast<int>(CommandID::CommandIDTag), command_result);

	RCLCPP_INFO(this->get_logger(),
	            "Successfully received tag info.");

	tag_publisher_->publish(tag_info_);
}

} // namespace uavrt_connection
