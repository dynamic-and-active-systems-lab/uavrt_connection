// Codebase for the Connection package used within the UAV-RT architecture.
// Copyright (C) 2023 Dynamic and Active Systems Lab
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

// ROS 2 interface header files
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/bool.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

// MAVSDK header files
#include "mavsdk/mavsdk.h"
#include "mavsdk/system.h"
#include "mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h"

// Project header files
#include "uavrt_interfaces/msg/pulse.hpp"
#include "uavrt_interfaces/msg/pulse_pose.hpp"
#include "uavrt_interfaces/msg/tag.hpp"
#include "uavrt_interfaces/TunnelProtocol.h"

namespace uavrt_connection
{

class CommandComponent : public rclcpp::Node
{
public:
    explicit CommandComponent(const rclcpp::NodeOptions &options,
                              std::shared_ptr<mavsdk::System> system);

private:
    // ROS 2 related - Private functions
    double TimeToDouble(int seconds, uint32_t nanoseconds);

    // MAVSDK related - Private functions
    void CommandCallback(const mavlink_message_t& message);

    void HandleAckCommand(uint32_t command_id, uint32_t result);

    void SendTunnelMessage(void* tunnel_payload, size_t tunnel_payload_size);
    void SendStatusText(const char* text);

    // ROS 2 and MAVSDK related - Private functions
    // These functions are acting as the bridges between ROS 2/MAVSDK (Mavlink) messages
    void HandleStartDetectionCommand();
    void HandleStopDetectionCommand();
    void HandlePulseCommand(uavrt_interfaces::msg::PulsePose::SharedPtr pulse_pose_message);
    void HandleTagCommand(const mavlink_tunnel_t& tunnel_message);
    void HandleStartTagCommand();
    void HandleEndTagCommand();

    // ROS 2 related - Private variables
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr start_subprocesses_publisher_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr stop_subprocesses_publisher_;

    rclcpp::Publisher<uavrt_interfaces::msg::Tag>::SharedPtr store_tag_information_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr release_tag_information_publisher_;

    rclcpp::Subscription<uavrt_interfaces::msg::PulsePose>::SharedPtr pulse_pose_subscriber_;

    int queue_size_ = 10;

    // For use in start command callback
    std_msgs::msg::Header start_command_header_;
    diagnostic_msgs::msg::DiagnosticArray start_command_diagnostic_array_;
    diagnostic_msgs::msg::DiagnosticStatus start_command_diagnostic_status_;
    diagnostic_msgs::msg::KeyValue start_command_key_value_;

    // For use in stop command callback
    std_msgs::msg::Header stop_command_header_;
    diagnostic_msgs::msg::DiagnosticArray stop_command_diagnostic_array_;
    diagnostic_msgs::msg::DiagnosticStatus stop_command_diagnostic_status_;
    diagnostic_msgs::msg::KeyValue stop_command_key_value_;

    // For use in tag command callback
    uavrt_interfaces::msg::Tag tag_info_;
    std_msgs::msg::Bool release_tag_information_bool_;

    // MAVSDK related - Private variables
    mavsdk::MavlinkPassthrough mavlink_passthrough_;

};

}  // namespace uavrt_connection

#endif  // UAVRT_CONNECTION_INCLUDE_UAVRT_CONNECTION_COMMAND_COMPONENT_HPP_
