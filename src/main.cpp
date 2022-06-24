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
// Note: Additional ROS 2 boilerplate code was sourced from the following ROS 2
// repository as well:
// https://github.com/ros2/examples/tree/galactic/rclcpp
//
// The associated documentation for the ROS 2 examples can be found
// throughout the "Tutorials" section on the ROS 2 documentation site:
// https://docs.ros.org/en/galactic/Tutorials.html
//
// The source code found within the aforementioned repositories is licensed under
// the Apache License, Version 2.0.

// C++ standard library headers
#include <memory>

// ROS 2 header files
#include "rclcpp/rclcpp.hpp"

// Project header files
#include "uavrt_connection/connection_handler.hpp"

int main(int argc, char * argv[])
{
	// Force flush of the stdout buffer.
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

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
	auto connection = std::make_shared<uavrt_connection::Connection>(options);
	exec.add_node(connection);

	// spin will block until work comes in, execute work as it becomes available, and keep blocking.
	// It will only be interrupted by Ctrl-C.
	exec.spin();

	rclcpp::shutdown();

	return 0;
}
