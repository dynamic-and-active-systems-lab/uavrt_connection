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

#ifndef UAVRT_CONNECTION_INCLUDE_UAVRT_CONNECTION_COMMAND_HANDLER_H_
#define UAVRT_CONNECTION_INCLUDE_UAVRT_CONNECTION_COMMAND_HANDLER_H_

// ROS 2 header files
#include "rclcpp/rclcpp.hpp"

// MAVSDK header files
#include "mavsdk/mavsdk.h"
#include "mavsdk/system.h"
#include "mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h"

// Project header files
#include "uavrt_interfaces/msg/tag_def.hpp"

namespace uavrt_connection
{

enum class CommandID
{
	// Ack response to command
	CommandIDAck = 1,
	// Tag info
	CommandIDTag = 2,
	// Detected pulse value
	CommandIDPulse = 3
};

enum class AckIndex
{
	// Command being acked
	AckIndexCommand = 0,
	// Command result - 1 success, 0 failure
	AckIndexResult = 2
};

enum class TagIndex
{
	// Tag id (uint 32)
	TagIndexID = 0,
	// Frequency - 6 digits shifted by three decimals, NNNNNN means NNN.NNN000 Mhz (uint 32)
	TagIndexFrequency = 1,
	// Pulse duration
	TagIndexDurationMSecs = 2,
	// Intra-pulse duration 1
	TagIndexIntraPulse1MSecs = 3,
	// Intra-pulse duration 2
	TagIndexIntraPulse2MSecs = 4,
	// Intra-pulse uncertainty
	TagIndexIntraPulseUncertainty = 5,
	// Intra-pulse jitter
	TagIndexIntraPulseJitter = 6,
	// Max pulse value
	TagIndexMaxPulse = 7
};

class CommandHandler
{
public:
explicit CommandHandler(std::shared_ptr<mavsdk::System> system,
                        rclcpp::Publisher<uavrt_interfaces::msg::TagDef>::SharedPtr tag_publisher_);

private:
bool CommandCallback(mavlink_message_t& message);

void HandleAckCommand(uint32_t command_id, uint32_t result);
void HandleTagCommand(const mavlink_debug_float_array_t& debugFloatArray);

mavsdk::MavlinkPassthrough mavlink_passthrough_;

rclcpp::Publisher<uavrt_interfaces::msg::TagDef>::SharedPtr tag_publisher_local_;

uavrt_interfaces::msg::TagDef tag_info_;

};

}  // namespace uavrt_connection

#endif  // UAVRT_CONNECTION_INCLUDE_UAVRT_CONNECTION_COMMAND_HANDLER_H_
