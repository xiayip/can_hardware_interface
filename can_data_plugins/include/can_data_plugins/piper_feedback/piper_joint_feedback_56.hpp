#pragma once

#include <map>
#include <utility>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include "can_data_plugins/can_data_base.hpp"

namespace can_data_plugins
{

struct PiperJointFeedback56 : public can_data_plugins::CanDataBase
{
    std::string prefix;

    double joint_5_actual_position = 0.0;
    double joint_6_actual_position = 0.0;

    bool initialize(hardware_interface::ComponentInfo &joint)
    {
        prefix = joint.parameters["prefix"];
        if (prefix.empty()) {
            prefix = "";
        }
        return true;
    }

    void export_state_interface(std::vector<hardware_interface::StateInterface> &state_interfaces)
    {
        RCLCPP_INFO(rclcpp::get_logger("PiperJointCtrlData"), "Exporting joint 5 and 6 state interfaces");
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(prefix + "joint5", hardware_interface::HW_IF_POSITION, &joint_5_actual_position));
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(prefix + "joint6", hardware_interface::HW_IF_POSITION, &joint_6_actual_position));
    }

    void export_command_interface(std::vector<hardware_interface::CommandInterface> &command_interfaces)
    {
    }

    void read_state(const int id, const uint8_t data[8])
    {
        float radian_to_degree = 180.0 / 3.14159265358979323846;
        // read joint 5 angle from data[0] to data[4]
        joint_5_actual_position = static_cast<double>((data[0] << 24)
            | (data[1] << 16) | (data[2] << 8) | data[3]) / (1000.0 * radian_to_degree);
        // read joint 6 angle from data[4] to data[7]
        joint_6_actual_position = static_cast<double>((data[4] << 24)
            | (data[5] << 16) | (data[6] << 8) | data[7]) / (1000.0 * radian_to_degree);
        // RCLCPP_INFO(rclcpp::get_logger("PiperJointCtrlData"), "Read state for ID %d: Joint 5: %.2f rad, Joint 6: %.2f rad",
        //     id, joint_5_actual_position, joint_6_actual_position);
    }

    bool write_target(const int id, uint8_t (&data)[8])
    {
        return false;
    }

    bool perform_switch(const std::vector<std::string> &start_interfaces,
                        const std::vector<std::string> &stop_interfaces)
    {
        return true;
    }
};
} // namespace can_data_plugins
