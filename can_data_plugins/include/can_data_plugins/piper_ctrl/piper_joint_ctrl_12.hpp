#pragma once

#include <map>
#include <utility>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include "can_data_plugins/can_data_base.hpp"

namespace can_data_plugins
{

struct PiperJointCtrl12 : public can_data_plugins::CanDataBase
{
    double joint_1_target_position = std::numeric_limits<double>::quiet_NaN();
    double joint_2_target_position = std::numeric_limits<double>::quiet_NaN();

    bool initialize(hardware_interface::ComponentInfo &joint)
    {
        return true;
    }

    void export_state_interface(std::vector<hardware_interface::StateInterface> &state_interfaces)
    {
    }

    void export_command_interface(std::vector<hardware_interface::CommandInterface> &command_interfaces)
    {
        RCLCPP_INFO(rclcpp::get_logger("PiperJointCtrl12"), "Exporting joint 1 and 2 command interfaces");
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface("joint1", hardware_interface::HW_IF_POSITION, &joint_1_target_position));
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface("joint2", hardware_interface::HW_IF_POSITION, &joint_2_target_position));
    }

    void read_state(const int id, const uint8_t data[8])
    {
    }

    bool write_target(const int id, uint8_t (&data)[8])
    {
        if (std::isnan(joint_1_target_position) || std::isnan(joint_2_target_position)) {
            // RCLCPP_WARN(rclcpp::get_logger("PiperJointCtrl12"), "Target positions are not set for joint 1 and 2.");
            return false;
        }

        float radian_to_degree = 180.0 / 3.14159265358979323846;
        // Convert target positions to CAN data format
        int32_t joint_1_target = static_cast<int32_t>(joint_1_target_position * (1000.0 * radian_to_degree));
        int32_t joint_2_target = static_cast<int32_t>(joint_2_target_position * (1000.0 * radian_to_degree));

        data[0] = (joint_1_target >> 24) & 0xFF;
        data[1] = (joint_1_target >> 16) & 0xFF;
        data[2] = (joint_1_target >> 8) & 0xFF;
        data[3] = joint_1_target & 0xFF;

        data[4] = (joint_2_target >> 24) & 0xFF;
        data[5] = (joint_2_target >> 16) & 0xFF;
        data[6] = (joint_2_target >> 8) & 0xFF;
        data[7] = joint_2_target & 0xFF;

        // RCLCPP_INFO(rclcpp::get_logger("PiperJointCtrl12"), "Write target for ID %03x: Joint 1: %.2f rad, Joint 2: %.2f rad",
        //     id, joint_1_target_position, joint_2_target_position);
        return true;
    }

    bool perform_switch(const std::vector<std::string> &start_interfaces,
                        const std::vector<std::string> &stop_interfaces)
    {
        return true;
    }
};
} // namespace can_data_plugins