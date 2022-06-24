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
#include <cinttypes>
#include <iostream>
#include <memory>

// ROS 2 header files
#include "rclcpp/rclcpp.hpp"

// Project header files
#include "uavrt_connection/connection_handler.hpp"

namespace uavrt_connection
{

Connection::Connection(const rclcpp::NodeOptions &options)
	: Node("Connection", options)
{
	RCLCPP_INFO(this->get_logger(), "Connection node");

	telemetry_timer_ = create_wall_timer(telemetry_period_ms_,
	                                     std::bind(&Connection::TelemetryCallback,
	                                               this));
}

void Connection::TelemetryCallback()
{
	RCLCPP_INFO(this->get_logger(), "telemetry_callback");
}

} // namespace uavrt_connection