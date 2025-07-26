#pragma once

#include <map>
#include <utility>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include "can_data_plugins/can_data_base.hpp"

namespace can_data_plugins
{

struct PiperArmStatus : public can_data_plugins::CanDataBase
{
    double ctrl_mode = std::numeric_limits<double>::quiet_NaN();
    double arm_status = std::numeric_limits<double>::quiet_NaN();
    double mode_feed = std::numeric_limits<double>::quiet_NaN();
    double teach_status  = std::numeric_limits<double>::quiet_NaN();
    double motion_status = std::numeric_limits<double>::quiet_NaN();
    double trajectory_num = std::numeric_limits<double>::quiet_NaN();
    double err_code = std::numeric_limits<double>::quiet_NaN();

    bool initialize(hardware_interface::ComponentInfo &joint)
    {
        return true;
    }

    void export_state_interface(std::vector<hardware_interface::StateInterface> &state_interfaces)
    {
        state_interfaces.emplace_back(
            hardware_interface::StateInterface("arm_status", "ctrl_mode", &ctrl_mode));
        state_interfaces.emplace_back(
            hardware_interface::StateInterface("arm_status", "arm_status", &arm_status));
        state_interfaces.emplace_back(
            hardware_interface::StateInterface("arm_status", "mode_feed", &mode_feed));
        state_interfaces.emplace_back(
            hardware_interface::StateInterface("arm_status", "teach_status", &teach_status));
        state_interfaces.emplace_back(
            hardware_interface::StateInterface("arm_status", "motion_status", &motion_status));
        state_interfaces.emplace_back(
            hardware_interface::StateInterface("arm_status", "trajectory_num", &trajectory_num));
        state_interfaces.emplace_back(
            hardware_interface::StateInterface("arm_status", "err_code", &err_code));
        RCLCPP_INFO(rclcpp::get_logger("PiperArmStatus"), "State interfaces exported for Piper Arm Status");
    }

    void export_command_interface(std::vector<hardware_interface::CommandInterface> &command_interfaces)
    {
    }

    void read_state(const int id, const uint8_t data[8])
    {
        ctrl_mode = static_cast<double>(data[0]);
        arm_status = static_cast<double>(data[1]);
        mode_feed = static_cast<double>(data[2]);
        teach_status = static_cast<double>(data[3]);
        motion_status = static_cast<double>(data[4]);
        trajectory_num = static_cast<double>(data[5]);
        err_code = static_cast<double>(data[6]);
        // RCLCPP_INFO(rclcpp::get_logger("PiperArmStatus"), "Read state for ID %03x: Ctrl Mode = %.2f, Arm Status = %.2f, Mode Feed = %.2f, "
        //     "Teach Status = %.2f, Motion Status = %.2f, Trajectory Num = %.2f, Error Code = %.2f",
        //     id, ctrl_mode, arm_status, mode_feed, teach_status, motion_status, trajectory_num, err_code);
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
