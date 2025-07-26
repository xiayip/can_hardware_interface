#pragma once

#include <map>
#include <utility>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include "can_data_plugins/can_data_base.hpp"

namespace can_data_plugins
{

struct PiperJointFeedback12 : public can_data_plugins::CanDataBase
{
    double joint_1_actual_position = 0.0;
    double joint_2_actual_position = 0.0;

    bool initialize(hardware_interface::ComponentInfo &joint)
    {
        return true;
    }

    void export_state_interface(std::vector<hardware_interface::StateInterface> &state_interfaces)
    {
        RCLCPP_INFO(rclcpp::get_logger("PiperJointCtrlData"), "Exporting joint 1 and 2 state interfaces");
        state_interfaces.emplace_back(
            hardware_interface::StateInterface("joint1", hardware_interface::HW_IF_POSITION, &joint_1_actual_position));
        state_interfaces.emplace_back(
            hardware_interface::StateInterface("joint2", hardware_interface::HW_IF_POSITION, &joint_2_actual_position));
    }

    void export_command_interface(std::vector<hardware_interface::CommandInterface> &command_interfaces)
    {
    }

    void read_state(const int id, const uint8_t data[8])
    {
        float radian_to_degree = 180.0 / 3.14159265358979323846;
        // read joint 1 angle from data[0] to data[4]
        joint_1_actual_position = static_cast<double>((data[0] << 24)
            | (data[1] << 16) | (data[2] << 8) | data[3]) / (1000.0 * radian_to_degree);
        // read joint 2 angle from data[4] to data[7]
        joint_2_actual_position = static_cast<double>((data[4] << 24)
            | (data[5] << 16) | (data[6] << 8) | data[7]) / (1000.0 * radian_to_degree);
        // RCLCPP_INFO(rclcpp::get_logger("PiperJointCtrlData"), "Read state for ID %d: Joint 1: %.2f rad, Joint 2: %.2f rad",
        //     id, joint_1_actual_position, joint_2_actual_position);
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