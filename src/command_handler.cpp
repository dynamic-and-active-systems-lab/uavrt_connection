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
#include <chrono>
#include <memory>

// Project header files
#include "uavrt_connection/command_handler.hpp"

namespace uavrt_connection
{

CommandHandler::CommandHandler(std::shared_ptr<mavsdk::System> system) :
	mavlink_passthrough_(system)
{
	mavlink_passthrough_.intercept_incoming_messages_async(std::bind(&CommandHandler::CommandCallback,
	                                                                  this,
	                                                                  std::placeholders::_1));
}

bool CommandHandler::CommandCallback(mavlink_message_t& message)
{
	if (message.msgid == MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY)
	{
		mavlink_debug_float_array_t debugFloatArray;

		mavlink_msg_debug_float_array_decode(&message, &debugFloatArray);

		switch (debugFloatArray.array_id)
		{
		case 1:
			// _handleTagCommand(debugFloatArray);
			break;
		}
	}

	// To drop a message, return 'false' from the callback.
	return true;
}

} // namespace uavrt_connection
