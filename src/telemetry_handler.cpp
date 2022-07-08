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

void TelemetryHandler::RefreshTelemetry(mavsdk::Telemetry mavsdk_telemetry)
{
	this->position_latitude_ = mavsdk_telemetry.position().latitude_deg;
	this->position_longitude_ = mavsdk_telemetry.position().longitude_deg;
	this->position_altitude_ = mavsdk_telemetry.position().absolute_altitude_m;
}

} // namespace uavrt_connection
