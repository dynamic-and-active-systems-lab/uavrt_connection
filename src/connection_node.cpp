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
#include <functional>
#include <memory>

// Project header files
#include "uavrt_connection/connection_node.hpp"

namespace uavrt_connection
{

// Every 'this' in this file is referring to the 'Connection' node initialized
// in the member initializer list.
// The Node constructor is passed the name of the node as well as
// applicable options. These are not needed here but are worth considering if
// using parameters.
// https://docs.ros2.org/galactic/api/rclcpp/classrclcpp_1_1NodeOptions.html
ConnectionNode::ConnectionNode(const rclcpp::NodeOptions& options)
	: Node("Connection", options), Link(), Telemetry(node_base)
{
	// In order to use rclcpp::Logger, you need to supply the get_logger()
	// function with a rclcpp::Node or you can call it while passing in the
	// name of the node. The second option does not check if the node actually
	// exists. AFAIK, this method of using get_logger() does not incur any
	// pentalies. This option is easier than passing in a pointer of
	// the node object within constructors or creating a seperate getter.
	// https://answers.ros.org/question/361542/ros-2-how-to-create-a-non-node-logger/
	RCLCPP_INFO(rclcpp::get_logger("Connection"), "Connection node successfully created.");

	// node_base = shared_from_this();
}

void ConnectionNode::initTelem()
{
	this->node_base = shared_from_this();
}

} // namespace uavrt_connection
