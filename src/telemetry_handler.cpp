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
#include <functional>
#include <memory>

// Project header files
#include "uavrt_connection/telemetry_handler.hpp"

namespace uavrt_connection
{

TelemetryHandler::TelemetryHandler(std::shared_ptr<mavsdk::System> system) :
	mavsdk_telemetry(system)
{
	mavsdk_telemetry.subscribe_position(std::bind(&TelemetryHandler::PositionCallback,
	                                              this,
	                                              std::placeholders::_1));
	mavsdk_telemetry.subscribe_attitude_quaternion(std::bind(&TelemetryHandler::QuaternionCallback,
	                                                         this,
	                                                         std::placeholders::_1));
}

double TelemetryHandler::GetPositionLatitude()
{
	return this->position_latitude_;
}

double TelemetryHandler::GetPositionLongitude()
{
	return this->position_longitude_;
}

float TelemetryHandler::GetPositionAltitude()
{
	return this->position_altitude_;
}

float TelemetryHandler::GetQuaternionW()
{
	return this->quaternion_w_;
}

float TelemetryHandler::GetQuaternionX()
{
	return this->quaternion_x_;
}

float TelemetryHandler::GetQuaternionY()
{
	return this->quaternion_y_;
}

float TelemetryHandler::GetQuaternionZ()
{
	return this->quaternion_z_;
}

// I'm not sure how to detect if there was an error getting the position
// attributes from the Telemetry object. E.g. the Pixhawk is unplugged, then
// the statements below should report that there was an issue.
// MAVSDK doesn't use exceptions but instead enum values. But I'm not sure
// what I'm supposed to be comparing to the enum value for success or failure.
// There isn't a result associated with the statements below.
// https://mavsdk.mavlink.io/main/en/cpp/guide/general_usage.html#error-handling
void TelemetryHandler::PositionCallback(mavsdk::Telemetry::Position position)
{
	// Poll for 'Position' (blocking).
	// https://mavsdk.mavlink.io/main/en/cpp/api_reference/classmavsdk_1_1_telemetry.html#classmavsdk_1_1_telemetry_1a61bda57b3ca47000ea7e4758b2a33134
	this->position_latitude_ = position.latitude_deg;
	this->position_longitude_ = position.longitude_deg;
	this->position_altitude_ = position.absolute_altitude_m;
}

void TelemetryHandler::QuaternionCallback(mavsdk::Telemetry::Quaternion quaternion)
{
    // Subscribe to 'attitude' updates (quaternion).
    // https://mavsdk.mavlink.io/main/en/cpp/api_reference/classmavsdk_1_1_telemetry.html#classmavsdk_1_1_telemetry_1afa6c079d48bc0c0a3287ac095ec290b9
	this->quaternion_w_ = quaternion.w;
	this->quaternion_x_ = quaternion.x;
	this->quaternion_y_ = quaternion.y;
	this->quaternion_z_ = quaternion.z;
}

} // namespace uavrt_connection
