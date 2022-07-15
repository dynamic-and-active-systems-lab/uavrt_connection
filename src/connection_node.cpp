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

//delete - only for debugging
#include <iostream>

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
ConnectionNode::ConnectionNode(const rclcpp::NodeOptions& options,
                               std::shared_ptr<mavsdk::System> system)
	: Node("ConnectionNode", options),
	TelemetryHandlerObject(system),
	CommandHandlerObject(system)
{
	// In order to use rclcpp::Logger, you need to supply the get_logger()
	// function with a rclcpp::Node or you can call it while passing in the
	// name of the node. The second option does not check if the node actually
	// exists. AFAIK, this method of using get_logger() does not incur any
	// pentalies. This option is easier than passing in a pointer of
	// the node object within constructors or creating a seperate getter.
	// https://answers.ros.org/question/361542/ros-2-how-to-create-a-non-node-logger/
	RCLCPP_INFO(rclcpp::get_logger("ConnectionNode"), "Connection node successfully created.");

	connection_status_ = true;

	system->subscribe_is_connected(std::bind(&ConnectionNode::ConnectionCallback,
	                                         this,
	                                         std::placeholders::_1));

	antenna_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
		"/antennaPose", queue_size_);
	antenna_pose_timer_ = this->create_wall_timer(
		antenna_pose_period_ms_, std::bind(&ConnectionNode::AntennaPoseCallback,
		                                   this));

}

// Search for the following terms in the link below for more info on "is_connected":
// typedef IsConnectedCallback, is_connected(), subscribe_is_connected()
// https://mavsdk.mavlink.io/main/en/cpp/api_reference/classmavsdk_1_1_system.html
void ConnectionNode::ConnectionCallback(bool is_connected)
{
	connection_status_ = is_connected;
	RCLCPP_ERROR(rclcpp::get_logger("ConnectionNode"),
	             "System has timed out! Attempting to reconnect.");
}

void ConnectionNode::AntennaPoseCallback()
{
	if (connection_status_ == true)
	{
		// http://docs.ros.org/en/lunar/api/std_msgs/html/msg/Header.html
		antenna_pose_header_ = std_msgs::msg::Header();
		antenna_pose_pose_stamped_ = geometry_msgs::msg::PoseStamped();
		// http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Pose.html
		antenna_pose_pose_ = geometry_msgs::msg::Pose();
		antenna_pose_point_ = geometry_msgs::msg::Point();
		antenna_pose_quaternion_ = geometry_msgs::msg::Quaternion();

		antenna_pose_header_.stamp = this->get_clock()->now();
		antenna_pose_header_.frame_id = "antenna_pose";

		antenna_pose_point_.x = TelemetryHandlerObject.GetPositionLatitude();
		antenna_pose_point_.y = TelemetryHandlerObject.GetPositionLongitude();
		antenna_pose_point_.z = TelemetryHandlerObject.GetPositionAltitude();

		antenna_pose_quaternion_.x = 0;
		antenna_pose_quaternion_.y = 0;
		antenna_pose_quaternion_.z = 0;
		antenna_pose_quaternion_.w = 0;

		antenna_pose_pose_.position = antenna_pose_point_;
		antenna_pose_pose_.orientation = antenna_pose_quaternion_;

		antenna_pose_pose_stamped_.header = antenna_pose_header_;
		antenna_pose_pose_stamped_.pose = antenna_pose_pose_;

		antenna_pose_publisher_->publish(antenna_pose_pose_stamped_);

		RCLCPP_INFO(rclcpp::get_logger("ConnectionNode"),
		            "Successfully published /antenna_pose.");

		// Still need to create an array for storing pose info.
	}
}

} // namespace uavrt_connection
