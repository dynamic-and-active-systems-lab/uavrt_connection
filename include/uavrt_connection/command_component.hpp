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

#ifndef UAVRT_CONNECTION_INCLUDE_UAVRT_CONNECTION_COMMAND_COMPONENT_HPP_
#define UAVRT_CONNECTION_INCLUDE_UAVRT_CONNECTION_COMMAND_COMPONENT_HPP_

// ROS 2 header files
#include "rclcpp/rclcpp.hpp"

// MAVSDK header files
#include "mavsdk/mavsdk.h"
#include "mavsdk/system.h"
#include "mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h"

// Project header files
#include "uavrt_interfaces/msg/pulse.hpp"
#include "uavrt_interfaces/msg/pulse_pose.hpp"
#include "uavrt_interfaces/msg/tag_def.hpp"
#include "uavrt_interfaces/qgc_enum_class_definitions.hpp"

namespace uavrt_connection
{

class CommandComponent : public rclcpp::Node
{
public:
explicit CommandComponent(const rclcpp::NodeOptions &options,
                          std::shared_ptr<mavsdk::System> system);

private:
// ROS 2 related - Private functions
void HandlePulseCommand(uavrt_interfaces::msg::PulsePose::SharedPtr pulse_pose_message);

// MAVSDK related - Private functions
bool CommandCallback(mavlink_message_t& message);

void HandleAckCommand(uint32_t command_id, uint32_t result);
void HandleTagCommand(const mavlink_debug_float_array_t& debugFloatArray);

// ROS 2 related - Private variables
rclcpp::Publisher<uavrt_interfaces::msg::TagDef>::SharedPtr tag_publisher_;

rclcpp::Subscription<uavrt_interfaces::msg::PulsePose>::SharedPtr pulse_pose_subscriber_;

int queue_size_ = 10;

uavrt_interfaces::msg::TagDef tag_info_;

// MAVSDK related - Private variables
mavsdk::MavlinkPassthrough mavlink_passthrough_;

};

}  // namespace uavrt_connection

#endif  // UAVRT_CONNECTION_INCLUDE_UAVRT_CONNECTION_COMMAND_COMPONENT_HPP_
