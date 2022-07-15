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

// Project header files
#include "uavrt_connection/telemetry_handler.hpp"

namespace uavrt_connection
{

TelemetryHandler::TelemetryHandler()
{

}

// I'm not sure how to detect if there was an error getting the position
// attributes from the Telemetry object. E.g. the Pixhawk is unplugged, then
// the statements below should report that there was an issue.
// MAVSDK doesn't use exceptions but instead enum values. But I'm not sure
// what I'm supposed to be comparing to the enum value for success or failure.
// There isn't a result associated with the statements below.
// https://mavsdk.mavlink.io/main/en/cpp/guide/general_usage.html#error-handling
void TelemetryHandler::RefreshTelemetry(mavsdk::Telemetry mavsdk_telemetry)
{
    // Poll for 'Position' (blocking).
    // https://mavsdk.mavlink.io/main/en/cpp/api_reference/classmavsdk_1_1_telemetry.html#classmavsdk_1_1_telemetry_1a2299da1bc63313c429f07ab0fdbe5335
	this->position_latitude_ = mavsdk_telemetry.position().latitude_deg;
	this->position_longitude_ = mavsdk_telemetry.position().longitude_deg;
	this->position_altitude_ = mavsdk_telemetry.position().absolute_altitude_m;
}

} // namespace uavrt_connection
