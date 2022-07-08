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
ConnectionNode::ConnectionNode(const rclcpp::NodeOptions& options)
	: Node("ConnectionNode", options), TelemetryHandlerObject()
{
	// In order to use rclcpp::Logger, you need to supply the get_logger()
	// function with a rclcpp::Node or you can call it while passing in the
	// name of the node. The second option does not check if the node actually
	// exists. AFAIK, this method of using get_logger() does not incur any
	// pentalies. This option is easier than passing in a pointer of
	// the node object within constructors or creating a seperate getter.
	// https://answers.ros.org/question/361542/ros-2-how-to-create-a-non-node-logger/
	RCLCPP_INFO(rclcpp::get_logger("ConnectionNode"), "Connection node successfully created.");

	// https://github.com/ros2/rclcpp/blob/galactic/rclcpp/include/rclcpp/rate.hpp
	rclcpp::Rate ros_sleep_rate(std::chrono::seconds(1));

	// Connection URL format should be:
	// For TCP : tcp://[server_host][:server_port]
	// For UDP : udp://[bind_host][:bind_port]
	// For Serial : serial:///path/to/serial/dev[:baudrate]
	// mavsdk.add_serial_connection() should work here as well, but I was
	// unable to connect to my serial device when using add_serial_connection().
	// https://mavsdk.mavlink.io/v0.33.0/en/api_reference/classmavsdk_1_1_mavsdk.html
	// For example, to connect to the Pixhawk use URL: "serial:///dev/ttyACM0"
	serial_mavsdk.add_any_connection("serial:///dev/ttyACM0");
	// For example, to connect to the simulator use URL: "udp://:14540"
	udp_mavsdk.add_udp_connection("udp://:14540");

	// The MAVSDK developers listed two possible ways to discover a system.
	// One method was using the C++ future library and the other was to use sleep.
	// I chose the latter since it's less complex and get's the job done.
	// But this method is blocking. It also incurs the issue of ROS 2 not being
	// able to shutdown if the system is stuck in the while loop.
	// We don't mind that it is blocking since it's vital that we establish
	// as system object initially. The while loop can be broken if we're
	// also watching the node to see if it had been issued the shutdown command.
	// However, an issue with this method is that that the node doesn't fully
	// shutdown after breaking from the while loop.
	// https://mavsdk.mavlink.io/main/en/cpp/api_changes.html#discovery-of-systems
	// https://docs.ros2.org/galactic/api/rclcpp/namespacerclcpp.html#adbe8ffd2b1769e897f2c50d560812b43
	while (serial_mavsdk.systems().size() == 0 &&
	       udp_mavsdk.systems().size() == 0 &&
	       rclcpp::ok() == true)
	{
		RCLCPP_ERROR(rclcpp::get_logger("ConnectionNode"), "MAVSDK connection failed.");
		ros_sleep_rate.sleep();
	}

	GetSystem();

	system_->subscribe_is_connected(std::bind(&ConnectionNode::ConnectionCallback,
	                                          this,
	                                          std::placeholders::_1));

	antenna_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
		"/antennaPose", queue_size_);
	antenna_pose_timer_ = this->create_wall_timer(
		antenna_pose_period_ms_, std::bind(&ConnectionNode::AntennaPoseCallback, this));

}

void ConnectionNode::GetSystem()
{
	// A serial connection will be used in the field but a UDP connection will
	// be necessary during indoors testing. This setup allows us to first check
	// if a serial option is available. If not, check UDP.
	if (serial_mavsdk.systems().size() == 1)
	{
		system_ = serial_mavsdk.systems().at(0);
		RCLCPP_INFO(rclcpp::get_logger("ConnectionNode"),
		            "Serial - MAVSDK connection successfully established.");
	}
	else if(udp_mavsdk.systems().size() == 1)
	{
		system_ = udp_mavsdk.systems().at(0);
		RCLCPP_INFO(rclcpp::get_logger("ConnectionNode"),
		            "UDP - MAVSDK connection successfully established.");
	}
	else
	{
		RCLCPP_ERROR(rclcpp::get_logger("ConnectionNode"), "MAVSDK connection failed.");
	}
}

// Search for the following terms in the link below for more info on "is_connected":
// typedef IsConnectedCallback, is_connected(), subscribe_is_connected()
// https://mavsdk.mavlink.io/main/en/cpp/api_reference/classmavsdk_1_1_system.html
void ConnectionNode::ConnectionCallback(bool is_connected)
{
	connection_status_ = is_connected;
	RCLCPP_ERROR(rclcpp::get_logger("ConnectionNode"),
	             "System has timed out! Attempting to reconnect.");
	// serial_mavsdk.systems().clear();
	// std::cout << "serial size: " << serial_mavsdk.systems().size() << std::endl;
	GetSystem();
}

void ConnectionNode::AntennaPoseCallback()
{
	if (connection_status_ == true)
	{
		// Not sure how to make this a member variable to avoid remaking it
		// each time.
		mavsdk::Telemetry mavsdk_telemetry = mavsdk::Telemetry(system_);

		TelemetryHandlerObject.RefreshTelemetry(mavsdk_telemetry);

		// http://docs.ros.org/en/lunar/api/std_msgs/html/msg/Header.html
		antenna_pose_header_ = std_msgs::msg::Header();
		antenna_pose_pose_stamped_ = geometry_msgs::msg::PoseStamped();
		// http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Pose.html
		antenna_pose_pose_ = geometry_msgs::msg::Pose();
		antenna_pose_point_ = geometry_msgs::msg::Point();
		antenna_pose_quaternion_ = geometry_msgs::msg::Quaternion();

		antenna_pose_header_.stamp = this->get_clock()->now();
		antenna_pose_header_.frame_id = "antenna_pose";

		antenna_pose_point_.x = TelemetryHandlerObject.position_latitude_;
		antenna_pose_point_.y = TelemetryHandlerObject.position_latitude_;
		antenna_pose_point_.z = TelemetryHandlerObject.position_latitude_;

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
