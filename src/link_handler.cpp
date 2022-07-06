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

// Project header files
#include "uavrt_connection/link_handler.hpp"

namespace uavrt_connection
{

LinkHandler::LinkHandler()
{
	SetSystem();
}

// In C++, methods and functions have the following syntax:
// <return-type> <class-name> :: <method-name> ( <arguments> ) { <statements> }
// Scope resolution operator (::)
void LinkHandler::SetSystem()
{
	mavsdk::Mavsdk mavsdk;

	// Connection URL format should be:
	// For TCP : tcp://[server_host][:server_port]
	// For UDP : udp://[bind_host][:bind_port]
	// For Serial : serial:///path/to/serial/dev[:baudrate]
    // For example, to connect to the Pixhawk use URL: "serial:///dev/ttyACM0"
	// For example, to connect to the simulator use URL: udp://:14540
	mavsdk::ConnectionResult connection_result =
		mavsdk.add_any_connection("serial:///dev/ttyACM0");

	if (connection_result != mavsdk::ConnectionResult::Success)
	{
		RCLCPP_ERROR(rclcpp::get_logger("Connection"),
		             "Serial connection failed.");
		return;
	}

	// Make this a member variable? How?
	static constexpr auto autopilot_timeout_s_ = std::chrono::seconds(1);

	// std::cout << "Waiting to discover system...\n";
	auto promise = std::promise<std::shared_ptr<mavsdk::System> >{};
	auto future = promise.get_future();

	// We wait for new systems to be discovered, once we find one that has an
	// autopilot, we decide to use it.
	mavsdk.subscribe_on_new_system([&mavsdk, &promise]()
		{
			auto system = mavsdk.systems().back();

			if (system->has_autopilot()) {

			    RCLCPP_INFO(rclcpp::get_logger("Connection"),
			                "Discovered autopilot.");

			    // Unsubscribe again as we only want to find one system.
			    mavsdk.subscribe_on_new_system(nullptr);
			    promise.set_value(system);
			}
		});

	// We usually receive heartbeats at 1Hz
	if (future.wait_for(autopilot_timeout_s_) ==
	    std::future_status::timeout)
	{
        RCLCPP_ERROR(rclcpp::get_logger("Connection"),
		             "No autopilot found.");
		return;
	}

	// Set the discovered system
	this->system_ = future.get();
}

std::shared_ptr<mavsdk::System> LinkHandler::GetSystem()
{
	return this->system_;
}

} // namespace uavrt_connection
