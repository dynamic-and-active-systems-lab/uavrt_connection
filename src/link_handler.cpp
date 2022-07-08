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
#include <future>
#include <thread>

// Project header files
#include "uavrt_connection/link_handler.hpp"

namespace uavrt_connection
{

LinkHandler::LinkHandler()
{
	// SetSystem();
}

// Search for the following terms in the link below for more info on "is_connected":
// typedef IsConnectedCallback, is_connected(), subscribe_is_connected()
// https://mavsdk.mavlink.io/main/en/cpp/api_reference/classmavsdk_1_1_system.html
void LinkHandler::MonitorLink(std::shared_ptr<mavsdk::System> system)
{
	RCLCPP_INFO(rclcpp::get_logger("Connection"),
	            "System has been discovered3.");
	system->subscribe_is_connected([this](bool is_connected)
		{
			if (is_connected)
			{
			    RCLCPP_INFO(rclcpp::get_logger("Connection"),
			                "System has been discovered2.");
			    this->StatusFLag = true;
			}
			else
			{
			    RCLCPP_ERROR(rclcpp::get_logger("Connection"),
			                 "System has timed out! Attempting to reconnect.");
			    this->StatusFLag = false;
			}
		});
}

// In C++, methods and functions have the following syntax:
// <return-type> <class-name> :: <method-name> ( <arguments> ) { <statements> }
// Scope resolution operator (::)
// MAVSDK developer's methodology to discovering new autopilot systems:
// https://mavsdk.mavlink.io/main/en/cpp/api_changes.html#discovery-of-systems
void LinkHandler::SetSystem()
{
	mavsdk::Mavsdk mavsdk;

	rclcpp::Rate sleepRate(std::chrono::seconds(1));

	// Connection URL format should be:
	// For TCP : tcp://[server_host][:server_port]
	// For UDP : udp://[bind_host][:bind_port]
	// For Serial : serial:///path/to/serial/dev[:baudrate]
	// For example, to connect to the Pixhawk use URL: "serial:///dev/ttyACM0"
	// For example, to connect to the simulator use URL: udp://:14540
	mavsdk::ConnectionResult connection_result =
		mavsdk.add_any_connection("serial:///dev/ttyACM0");

	// if (connection_result != mavsdk::ConnectionResult::Success)
	// {
	// 	RCLCPP_ERROR(rclcpp::get_logger("Connection"), "Serial connection failed.");
	// 	return;
	// }

	while (connection_result != mavsdk::ConnectionResult::Success)
	{
		RCLCPP_ERROR(rclcpp::get_logger("Connection"), "Serial connection failed.");
		sleepRate.sleep();
	}

	// Make this a member variable? How?
	static constexpr auto autopilot_timeout_s_ = std::chrono::seconds(1);

	// std::cout << "Waiting to discover system...\n";
	auto system_promise = std::promise<std::shared_ptr<mavsdk::System> >{};
	auto system_future = system_promise.get_future();

	// We wait for new systems to be discovered, once we find one that has an
	// autopilot, we decide to use it.
	mavsdk.subscribe_on_new_system([&mavsdk, &system_promise]()
		{
			auto system = mavsdk.systems().back();

			if (system->has_autopilot())
			{
			    RCLCPP_INFO(rclcpp::get_logger("Connection"), "Discovered autopilot.");

			    // Unsubscribe again as we only want to find one system.
			    mavsdk.subscribe_on_new_system(nullptr);
			    system_promise.set_value(system);
			}
		});

	// We usually receive heartbeats at 1Hz
	if (system_future.wait_for(autopilot_timeout_s_) == std::future_status::timeout)
	{
		RCLCPP_ERROR(rclcpp::get_logger("Connection"), "No autopilot found.");
		return;
	}

	auto system = system_future.get();

	system->subscribe_is_connected([](bool is_connected)
		{
			if (is_connected)
			{
			    RCLCPP_INFO(rclcpp::get_logger("Connection"),
			                "System has been discovered2.");
			}
			else
			{
			    RCLCPP_ERROR(rclcpp::get_logger("Connection"),
			                 "System has timed out! Attempting to reconnect.");
			}
		});

	RCLCPP_ERROR(rclcpp::get_logger("Connection"), "No autopilot found.");

	// while (system->is_connected() == true)
	// {
	// 	std::this_thread::sleep_for(std::chrono::seconds(1));
	// }
	// Set the discovered system
	// this->system_ = system_future.get();
}

void LinkHandler::SetTelemetry()
{
	// mavsdk::Telemetry telemetry = mavsdk::Telemetry{this->system_};
}

} // namespace uavrt_connection
