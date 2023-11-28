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

// C++ standard library headers
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <chrono>

// Project header files
#include "uavrt_connection/command_component.hpp"

namespace uavrt_connection
{

CommandComponent::CommandComponent(const rclcpp::NodeOptions& options,
                                   std::shared_ptr<mavsdk::System> system)
	: Node("CommandComponent", options),
	  mavlink_passthrough_(system)
{
	RCLCPP_INFO(this->get_logger(), "Command component successfully created.");

	// Check that TunnelProtocol hasn't exceed limits
	static_assert(TunnelProtocolValidateSizes, "TunnelProtocolValidateSizes failed");

	// ROS 2 related - Publisher callback
	start_subprocesses_publisher_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
	                                    "control_start_subprocess",
	                                    queue_size_);

	// ROS 2 related - Publisher callback
	stop_subprocesses_publisher_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
	                                   "control_stop_subprocess",
	                                   queue_size_);

	// ROS 2 related - Publisher callback
	store_tag_information_publisher_ = this->create_publisher<uavrt_interfaces::msg::Tag>(
	                                       "store_tag_information",
	                                       queue_size_);

	// ROS 2 related - Publisher callback
	release_tag_information_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
                                	        "release_tag_information",
                                	        queue_size_);

	// ROS 2 related - Subscriber callback
	pulse_pose_subscriber_ = this->create_subscription<uavrt_interfaces::msg::PulsePose>(
	                             "pulse_pose", queue_size_, std::bind(&CommandComponent::HandlePulseCommand,
	                             this,
	                             std::placeholders::_1));

	// MAVSDK related
	mavlink_passthrough_.subscribe_message_async(MAVLINK_MSG_ID_TUNNEL,
	        std::bind(&CommandComponent::CommandCallback,
	                  this,
	                  std::placeholders::_1));
}

void CommandComponent::CommandCallback(const mavlink_message_t& message)
{
	mavlink_tunnel_t tunnel;

	mavlink_msg_tunnel_decode(&message, &tunnel);

	TunnelProtocol::HeaderInfo_t header_info;

	if (tunnel.payload_length < sizeof(header_info))
	{
		RCLCPP_ERROR(this->get_logger(),
		             "Tunnel message payload too small.");
		return;
	}

	memcpy(&header_info, tunnel.payload, sizeof(header_info));

	switch (header_info.command)
	{
	case COMMAND_ID_START_DETECTION:
		HandleStartDetectionCommand();
		break;
	case COMMAND_ID_STOP_DETECTION:
		HandleStopDetectionCommand();
		break;
	case COMMAND_ID_TAG:
		HandleTagCommand(tunnel);
		break;
	case COMMAND_ID_START_TAGS:
		HandleStartTagCommand();
		break;
	case COMMAND_ID_END_TAGS:
		HandleEndTagCommand();
		break;
	}
}

void CommandComponent::HandleAckCommand(uint32_t command_id, uint32_t result)
{
	TunnelProtocol::AckInfo_t ack_info;

	RCLCPP_INFO(this->get_logger(),
	            "Sending back ack message for command: %d", command_id);

	ack_info.header.command  = COMMAND_ID_ACK;
	ack_info.command         = command_id;
	ack_info.result          = result;

	SendTunnelMessage(&ack_info, sizeof(ack_info));
}

void CommandComponent::HandleStartDetectionCommand()
{
	uint32_t command_result = COMMAND_RESULT_SUCCESS;

	// https://docs.ros2.org/galactic/api/std_msgs/msg/Header.html
	start_command_header_ = std_msgs::msg::Header();
	// https://docs.ros2.org/galactic/api/diagnostic_msgs/msg/DiagnosticArray.html
	start_command_diagnostic_array_ = diagnostic_msgs::msg::DiagnosticArray();
	// https://docs.ros2.org/galactic/api/diagnostic_msgs/msg/DiagnosticStatus.html
	start_command_diagnostic_status_ = diagnostic_msgs::msg::DiagnosticStatus();
	// https://docs.ros2.org/galactic/api/diagnostic_msgs/msg/KeyValue.html
	start_command_key_value_ = diagnostic_msgs::msg::KeyValue();

	// https://docs.ros2.org/galactic/api/builtin_interfaces/msg/Time.html
	start_command_header_.stamp = this->get_clock()->now();
	start_command_header_.frame_id = "start_command_subprocesses";

	start_command_diagnostic_status_.level = '0';
	start_command_diagnostic_status_.name = "NA";
	start_command_diagnostic_status_.message = "start";
	start_command_diagnostic_status_.hardware_id = "start all";

	start_command_diagnostic_array_.status.push_back(start_command_diagnostic_status_);

	start_command_diagnostic_array_.header = start_command_header_;

	RCLCPP_INFO(this->get_logger(),
	            "Successfully received start command.");

	start_subprocesses_publisher_->publish(start_command_diagnostic_array_);

	HandleAckCommand(COMMAND_ID_START_DETECTION, command_result);
}

void CommandComponent::HandleStopDetectionCommand()
{
	uint32_t command_result = COMMAND_RESULT_SUCCESS;

	stop_command_header_ = std_msgs::msg::Header();
	stop_command_diagnostic_array_ = diagnostic_msgs::msg::DiagnosticArray();
	stop_command_diagnostic_status_ = diagnostic_msgs::msg::DiagnosticStatus();
	stop_command_key_value_ = diagnostic_msgs::msg::KeyValue();

	stop_command_header_.stamp = this->get_clock()->now();
	stop_command_header_.frame_id = "stop_command_subprocesses";

	stop_command_diagnostic_status_.level = '0';
	stop_command_diagnostic_status_.name = "NA";
	stop_command_diagnostic_status_.message = "stop";
	stop_command_diagnostic_status_.hardware_id = "stop all";

	stop_command_diagnostic_array_.status.push_back(stop_command_diagnostic_status_);

	stop_command_diagnostic_array_.header = stop_command_header_;

	RCLCPP_INFO(this->get_logger(),
	            "Successfully received stop command.");

	stop_subprocesses_publisher_->publish(stop_command_diagnostic_array_);

	HandleAckCommand(COMMAND_ID_STOP_DETECTION, command_result);
}

void CommandComponent::HandleTagCommand(const mavlink_tunnel_t& tunnel)
{
	TunnelProtocol::TagInfo_t new_tag_info;
	uint32_t command_result = COMMAND_RESULT_SUCCESS;

	memcpy(&new_tag_info, tunnel.payload, sizeof(new_tag_info));

	// https://github.com/dynamic-and-active-systems-lab/uavrt_interfaces/blob/main/msg/TagDef.msg
	tag_info_ = uavrt_interfaces::msg::Tag();

	tag_info_.tag_id = new_tag_info.id;
	tag_info_.frequency = new_tag_info.frequency_hz;
	tag_info_.pulse_duration = new_tag_info.pulse_width_msecs;
	tag_info_.interpulse_time_1 = new_tag_info.intra_pulse1_msecs;
	tag_info_.interpulse_time_2 = new_tag_info.intra_pulse2_msecs;
	tag_info_.interpulse_time_uncert = new_tag_info.intra_pulse_uncertainty_msecs;
	tag_info_.interpulse_time_jitter = new_tag_info.intra_pulse_jitter_msecs;
	tag_info_.k = new_tag_info.k;
	tag_info_.false_alarm_probability = new_tag_info.false_alarm_probability;

	if (tag_info_.tag_id == 0)
	{
		RCLCPP_ERROR(this->get_logger(), "Invalid tag id of 0.");
		command_result  = COMMAND_RESULT_FAILURE;
	}
	else if (tag_info_.tag_id != 0)
	{
		RCLCPP_INFO(this->get_logger(),
		            "Successfully received tag info.");

		store_tag_information_publisher_->publish(tag_info_);
	}

	HandleAckCommand(COMMAND_ID_TAG, command_result);
}

void CommandComponent::HandlePulseCommand(
    uavrt_interfaces::msg::PulsePose::SharedPtr pulse_pose_message)
{
	TunnelProtocol::PulseInfo_t pulse_info;

	// Refer to uavrt_interfaces/include/uavrt_interfaces/TunnelProtocol.h
	// for descriptions on each of the data values.
	pulse_info.header.command = COMMAND_ID_PULSE;
	pulse_info.tag_id = pulse_pose_message->pulse.tag_id;
	pulse_info.frequency_hz = pulse_pose_message->pulse.frequency;
	pulse_info.start_time_seconds = TimeToDouble(pulse_pose_message->pulse.start_time.sec,
	                                pulse_pose_message->pulse.start_time.nanosec);
	pulse_info.predict_next_start_seconds = TimeToDouble(pulse_pose_message->pulse.predict_next_start.sec,
	                                        pulse_pose_message->pulse.predict_next_start.nanosec);
	pulse_info.snr = pulse_pose_message->pulse.snr;
	pulse_info.stft_score = pulse_pose_message->pulse.stft_score;
	pulse_info.group_ind = pulse_pose_message->pulse.group_ind;
	pulse_info.group_snr = pulse_pose_message->pulse.group_snr;
	pulse_info.detection_status = pulse_pose_message->pulse.detection_status;
	pulse_info.confirmed_status = pulse_pose_message->pulse.confirmed_status;

	pulse_info.position_x = pulse_pose_message->antenna_pose.position.x;
	pulse_info.position_y = pulse_pose_message->antenna_pose.position.y;
	pulse_info.position_z = pulse_pose_message->antenna_pose.position.z;

	pulse_info.orientation_x = pulse_pose_message->antenna_pose.orientation.x;
	pulse_info.orientation_y = pulse_pose_message->antenna_pose.orientation.y;
	pulse_info.orientation_z = pulse_pose_message->antenna_pose.orientation.z;
	pulse_info.orientation_w = pulse_pose_message->antenna_pose.orientation.w;

	SendTunnelMessage(&pulse_info, sizeof(pulse_info));

	// Delay to avoid tunnel messages being dropped by MAVLINK.
	std::this_thread::sleep_for(packet_delay_time_ms);

	RCLCPP_INFO(this->get_logger(),
	            "Successfully sent pulse pose message to the ground.");
}

// Currently unused. We send back an ack to avoid errors on QGC's side.
void CommandComponent::HandleStartTagCommand()
{
	RCLCPP_INFO(this->get_logger(),
	            "Successfully received start tag command.");

	HandleAckCommand(COMMAND_ID_START_TAGS, COMMAND_RESULT_SUCCESS);
}

void CommandComponent::HandleEndTagCommand()
{
	RCLCPP_INFO(this->get_logger(),
	            "Successfully received end tag command.");

	release_tag_information_bool_ = std_msgs::msg::Bool();

	release_tag_information_bool_.data = true;

	release_tag_information_publisher_->publish(release_tag_information_bool_);

	HandleAckCommand(COMMAND_ID_END_TAGS, COMMAND_RESULT_SUCCESS);
}

void CommandComponent::SendTunnelMessage(void* tunnel_payload,
        size_t tunnel_payload_size)
{
	mavlink_message_t message;
	mavlink_tunnel_t tunnel;

	memset(&tunnel, 0, sizeof(tunnel));

	tunnel.target_system    = mavlink_passthrough_.get_target_sysid();
	tunnel.target_component = mavlink_passthrough_.get_target_compid();
	tunnel.payload_type     = MAV_TUNNEL_PAYLOAD_TYPE_UNKNOWN;
	tunnel.payload_length   = tunnel_payload_size;

	memcpy(tunnel.payload, tunnel_payload, tunnel_payload_size);

	mavlink_msg_tunnel_encode(
	    mavlink_passthrough_.get_our_sysid(),
	    mavlink_passthrough_.get_our_compid(),
	    &message,
	    &tunnel);

	mavlink_passthrough_.send_message(message);
}

// TODO: Need to change this function to be a ROS 2 subsriber.
// Subsribe to status_messages, grab the text from the message, and then
// send down along with other ROS 2 diagnostic_msg info.
void CommandComponent::SendStatusText(const char* text)
{
	/*
	    mavlink_message_t message;
	    mavlink_statustext_t statustext;

	    memset(&statustext, 0, sizeof(statustext));

	    // Info shows up under megaphone
	    // Warning and above shows as pop up
	    statustext.severity = MAV_SEVERITY_INFO;

	    strncpy(statustext.text, text, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);

	    mavlink_msg_statustext_encode(
	            mavlink_passthrough_.get_our_sysid(),
	            mavlink_passthrough_.get_our_compid(),
	            &message,
	            &statustext);

	    mavlink_passthrough_.send_message(message);
	 */
}

// Helper function for values that can not be inherently converted to doubles
// (like string and ROS 2 builtin_interfaces/Time). These values need to be
// manually converted.
double CommandComponent::TimeToDouble(int seconds, uint32_t nanoseconds)
{
	std::string uncoverted_time_value =
	    std::to_string(seconds) + "." +
	    std::to_string(nanoseconds);

	double converted_time_value = std::stod(uncoverted_time_value);

	return converted_time_value;
}
} // namespace uavrt_connection
