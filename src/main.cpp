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
//
// Note: A majority of the ROS 2 boilerplate code in this file and subsequent
// files was sourced from the demos hosted within this ROS 2 repository:
// https://github.com/ros2/demos/tree/galactic
//
// Specifically this directory within the ros2/demos repository:
// https://github.com/ros2/demos/tree/galactic/composition
//
// The associated documentation for ROS 2 demos can be found here:
// https://docs.ros.org/en/galactic/Tutorials/Demos.html
//
// Additional ROS 2 boilerplate code was sourced from the following ROS 2
// repository as well:
// https://github.com/ros2/examples/tree/galactic/rclcpp
//
// The associated documentation for the ROS 2 examples can be found
// throughout the "Tutorials" section on the ROS 2 documentation site:
// https://docs.ros.org/en/galactic/Tutorials.html
//
// The source code found within the aforementioned repositories is licensed under
// the Apache License, Version 2.0.
//
// Note: A majority of the MAVSDK boilerplate code in this subsequent
// files was sourced from the examples hosted within this MAVSDK repository:
// https://github.com/mavlink/MAVSDK/tree/main/examples
//
// The associated documentation for MAVSDK examples can be found here:
// https://mavsdk.mavlink.io/main/en/cpp/examples/
//
// It is unclear as to whether the example code falls under the BSD 3-Clause
// license used by the MAVSDK developers.

// C++ standard library headers
#include <memory>
#include <future>
#include <chrono>
#include <string>

// ROS 2 header files
#include "rclcpp/rclcpp.hpp"

// MAVSDK header files
#include "mavsdk/mavsdk.h"
#include "mavsdk/system.h"

// Project header files
#include "uavrt_connection/connection_node.hpp"

static void usage()
{
	std::cerr << "Usage : Enter a '0' to use a serial connection "
	          << "or enter a '1' to use an UDP connection. \n"
	          << "For Serial, the connection string will be: 'serial:///dev/ttyACM0', \n"
	          << "and expects a PX4 autopilot to be plugged into this port. \n"
	          << "For UDP, the connection string will be: 'udp://:14540', \n"
	          << "and expects a Gazebo SITL to be running at this port. \n";
}

std::shared_ptr<mavsdk::System> get_system(mavsdk::Mavsdk& mavsdk)
{
	RCLCPP_INFO(rclcpp::get_logger("Main"), "Waiting to discover system...");
	auto system_promise = std::promise<std::shared_ptr<mavsdk::System> >{};
	auto system_future = system_promise.get_future();

	// We wait for new systems to be discovered, once we find one that has an
	// autopilot, we decide to use it.
	mavsdk.subscribe_on_new_system([&mavsdk, &system_promise]()
	{
		auto system = mavsdk.systems().back();

		if (system->has_autopilot()) {
		    RCLCPP_INFO(rclcpp::get_logger("Main"), "Discovered autopilot.");

		    // Unsubscribe again as we only want to find one system.
		    mavsdk.subscribe_on_new_system(nullptr);
		    system_promise.set_value(system);
		}
	});

	// We usually receive heartbeats at 1Hz
	// This value should be greater than 1
	if (system_future.wait_for(std::chrono::seconds(5)) ==
	    std::future_status::timeout)
	{
		RCLCPP_ERROR(rclcpp::get_logger("Main"), "No autopilot found.");
		return {};
	}

	// Get discovered system now.
	return system_future.get();
}


int main(int argc, char *argv[])
{
	// Force flush of the stdout buffer.
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

	// Exit the program if the user does not input the necessary arguement.
	if (argc != 2) { usage(); return 1; }

	int arg_val = std::stoi(argv[1]);

	mavsdk::Mavsdk mavsdk;
	mavsdk::ConnectionResult connection_result;

	// Connection URL format should be:
	// For TCP : tcp://[server_host][:server_port]
	// For UDP : udp://[bind_host][:bind_port]
	// For Serial : serial:///path/to/serial/dev[:baudrate]
	// mavsdk.add_serial_connection() should work here as well, but I was
	// unable to connect to my serial device when using add_serial_connection().
	// https://mavsdk.mavlink.io/v0.33.0/en/api_reference/classmavsdk_1_1_mavsdk.html
	// For example, to connect to the PX4 autopilot use URL: "serial:///dev/ttyACM0"
	// For example, to connect to the Gazebo SITL use URL: "udp://:14540"
	if (arg_val == 0)
	{ connection_result = mavsdk.add_any_connection("serial:///dev/ttyACM0"); }
	else if (arg_val == 1)
	{ connection_result = mavsdk.add_any_connection("udp://:14540"); }
	else
	{
		RCLCPP_ERROR(rclcpp::get_logger("Main"), "Illegal input.");
		return 1;
	}

	if (connection_result != mavsdk::ConnectionResult::Success)
	{
		RCLCPP_ERROR(rclcpp::get_logger("Main"), "Connection failed.");
		return 1;
	}

	std::shared_ptr<mavsdk::System> system = get_system(mavsdk);

	if (!system)
	{
		RCLCPP_ERROR(rclcpp::get_logger("Main"), "System object is not valid.");
		return 1;
	}

	// Initialize any global resources needed by the middleware and the client library.
	// This will also parse command line arguments one day (as of (ROS 2) Beta 1 they are not used).
	// You must call this before using any other part of the ROS system.
	// This should be called once per process.
	// Command line arguments are not necessary for use within the uavrt_connection package.
	// argc and argv are left here to maintain coherence with the ROS 2 boilerplate code.
	rclcpp::init(argc, argv);

	// Create an executor that will be responsible for execution of callbacks for a set of nodes.
	// With this version, all callbacks will be called from within this thread (the main one).
	rclcpp::executors::SingleThreadedExecutor exec;
	rclcpp::NodeOptions options;

	// Add some nodes to the executor which provide work for the executor during its "spin" function.
	// An example of available work is executing a subscription callback, or a timer callback.
	auto connection = std::make_shared<uavrt_connection::ConnectionNode>(options,
	                                                                     system);
	exec.add_node(connection);

	// spin will block until work comes in, execute work as it becomes available, and keep blocking.
	// It will only be interrupted by Ctrl-C.
	exec.spin();

	rclcpp::shutdown();

	return 0;
}
