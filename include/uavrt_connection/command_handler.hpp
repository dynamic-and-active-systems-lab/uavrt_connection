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

namespace uavrt_connection
{

enum class CommandID
{
	CommandIDAck = 1,         // Ack response to command
	CommandIDTag = 2,         // Tag info
	CommandIDPulse = 3         // Detected pulse value
};

enum class AckIndex
{
	AckIndexCommand = 0,         // Command being acked
	AckIndexResult = 2         // Command result - 1 success, 0 failure
};

class CommandHandler
{
public:
explicit CommandHandler(std::shared_ptr<mavsdk::System> system);

private:
bool CommandCallback(mavlink_message_t& message);

mavsdk::MavlinkPassthrough mavlink_passthrough_;

};

}  // namespace uavrt_connection

#endif  // UAVRT_CONNECTION_INCLUDE_UAVRT_CONNECTION_COMMAND_HANDLER_H_
