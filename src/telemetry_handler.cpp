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
#include "uavrt_connection/telemetry_handler.hpp"

namespace uavrt_connection
{

TelemetryHandler::TelemetryHandler(rclcpp::Node::SharedPtr& node)
{
	telemetry_timer_ = rclcpp::create_wall_timer(telemetry_period_ms_,
	                                             std::bind(&TelemetryHandler::TelemetryCallback,
	                                                       node));
}

void TelemetryHandler::TelemetryCallback()
{
	RCLCPP_INFO(rclcpp::get_logger("Connection"),
	            "Telemetry callback.");
}
} // namespace uavrt_connection
