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

#ifndef UAVRT_CONNECTION_INCLUDE_UAVRT_CONNECTION_TELEMETRY_HANDLER_H_
#define UAVRT_CONNECTION_INCLUDE_UAVRT_CONNECTION_TELEMETRY_HANDLER_H_

// ROS 2 header files
#include "rclcpp/rclcpp.hpp"

// MAVSDK header files
// MAVSDK header files
#include "mavsdk/mavsdk.h"
#include "mavsdk/system.h"
#include "mavsdk/plugins/telemetry/telemetry.h"

namespace uavrt_connection
{

class TelemetryHandler
{
public:
explicit TelemetryHandler(std::shared_ptr<mavsdk::System> system);

double GetPositionLatitude();
double GetPositionLongitude();
float GetPositionAltitude();

float GetQuaternionW();
float GetQuaternionX();
float GetQuaternionY();
float GetQuaternionZ();

private:
void PositionCallback(mavsdk::Telemetry::Position position);
void QuaternionCallback(mavsdk::Telemetry::Quaternion quaternion);

mavsdk::Telemetry mavsdk_telemetry;

// https://mavsdk.mavlink.io/main/en/cpp/api_reference/structmavsdk_1_1_telemetry_1_1_position.html#structmavsdk_1_1_telemetry_1_1_position_1
double position_latitude_;     // Latitude in degrees (range: -90 to +90)
double position_longitude_;     // Longitude in degrees (range: -180 to +180)
float position_altitude_;     // Altitude AMSL (above mean sea level) in metres

// https://mavsdk.mavlink.io/main/en/cpp/api_reference/structmavsdk_1_1_telemetry_1_1_quaternion.html
float quaternion_w_;
float quaternion_x_;
float quaternion_y_;
float quaternion_z_;

};

}  // namespace uavrt_connection

#endif  // UAVRT_CONNECTION_INCLUDE_UAVRT_CONNECTION_TELEMETRY_HANDLER_H_
