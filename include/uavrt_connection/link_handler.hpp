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

#ifndef UAVRT_CONNECTION_INCLUDE_UAVRT_CONNECTION_LINK_HANDLER_H_
#define UAVRT_CONNECTION_INCLUDE_UAVRT_CONNECTION_LINK_HANDLER_H_

// ROS 2 header files
#include "rclcpp/rclcpp.hpp"

// MAVSDK header files
#include "mavsdk/mavsdk.h"
#include "mavsdk/system.h"
#include "mavsdk/plugins/telemetry/telemetry.h"

namespace uavrt_connection
{

class LinkHandler
{
public:
explicit LinkHandler();

bool StatusFLag;

void SetSystem();

void MonitorLink(std::shared_ptr<mavsdk::System> system);

private:


void SetTelemetry();


std::shared_ptr<mavsdk::System> system_;

};

}  // namespace uavrt_connection

#endif  // UAVRT_CONNECTION_INCLUDE_UAVRT_CONNECTION_LINK_HANDLER_H_
