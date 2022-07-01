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

// Note: Header files from other packages/libraries have to be added to
// uavrt_connection/package.xml and/or uavrt_connection/CMakeLists.txt
// ROS 2 header files
#include "rclcpp/rclcpp.hpp"

// Project header files
#include "uavrt_connection/connection_handler.hpp"

namespace uavrt_connection
{

// Create the node class Connection by inheriting from rclcpp::Node.
// Every 'this' in the code is referring to the this Connection node.
// The member initializer list is passed the name of the node as well as
// necessary options. These are not necessary but are worth considering if
// using parameters.
// https://docs.ros2.org/galactic/api/rclcpp/classrclcpp_1_1NodeOptions.html
Connection::Connection(const rclcpp::NodeOptions &options)
	: Node("Connection", options)
{
	RCLCPP_INFO(this->get_logger(), "Connection node successfully created.");

	telemetry_timer_ = create_wall_timer(telemetry_period_ms_,
	                                     std::bind(&Connection::TelemetryCallback,
	                                               this));
}

void Connection::TelemetryCallback()
{
	RCLCPP_INFO(this->get_logger(), "telemetry_callback");
}

} // namespace uavrt_connection
