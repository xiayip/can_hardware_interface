#pragma once

#include <map>
#include <utility>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include "can_data_plugins/can_data_base.hpp"

/*
    CAN ID:
        0x151
    
    Args:
        ctrl_mode: Control mode.
        move_mode: MOVE mode.
        move_spd_rate_ctrl: Movement speed as a percentage.
        mit_mode: MIT mode.
        residence_time: Hold time at offline trajectory points.

    Bit Descriptions:

        Byte 0 control_mode: uint8, control mode selection.
            0x00: Standby mode.
            0x01: CAN command control mode.
            0x03: Ethernet control mode.
            0x04: Wi-Fi control mode.
            0x07: Offline trajectory mode.

        Byte 1 move_mode: uint8, movement mode selection.
            0x00: MOVE P (Position).
            0x01: MOVE J (Joint).
            0x02: MOVE L (Linear).
            0x03: MOVE C (Circular).
            0x04: MOVE M ---基于V1.5-2版本后

        Byte 2 speed_percentage: uint8, movement speed as a percentage.
            Range: 0~100.

        Byte 3 mit_mode: uint8, motion control mode.
            0x00: Position-speed mode.
            0xAD: MIT mode.
            0xFF: Invalid.

        Byte 4 offline_trajectory_hold_time: uint8, duration to hold at offline trajectory points.
            Range: 0~255, unit: seconds.
*/

namespace can_data_plugins
{

struct PiperArmMotionCtrl2 : public can_data_plugins::CanDataBase
{
    double control_mode = std::numeric_limits<double>::quiet_NaN();
    double move_mode = std::numeric_limits<double>::quiet_NaN();
    double move_spd_rate_ctrl = std::numeric_limits<double>::quiet_NaN();
    double mit_mode = std::numeric_limits<double>::quiet_NaN();

    bool initialize(hardware_interface::ComponentInfo &joint)
    {
        control_mode = 1.0;
        move_spd_rate_ctrl = 100.0; // 100%
        mit_mode = joint.parameters.find("mit_mode") != joint.parameters.end()
                                ? std::stod(joint.parameters["mit_mode"])
                                : 0.0;
        if (mit_mode == 0x00) {
            move_mode = 1.0; // MOVE J
            RCLCPP_INFO(rclcpp::get_logger("PiperArmMotionCtrl2"), "MIT mode is set to 0x00 (0)");
        } else if (mit_mode == 0xAD) {
            move_mode = 4.0; // MOVE M
            RCLCPP_INFO(rclcpp::get_logger("PiperArmMotionCtrl2"), "MIT mode is set to 0xAD (173)");
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("PiperArmMotionCtrl2"), "Invalid MIT mode: %f", mit_mode);
            return false;
        }
        return true;
    }

    void export_state_interface(std::vector<hardware_interface::StateInterface> &state_interfaces)
    {
    }

    void export_command_interface(std::vector<hardware_interface::CommandInterface> &command_interfaces)
    {
    }

    void read_state(const int id, const uint8_t data[8])
    {
    }

    bool write_target(const int id, uint8_t (&data)[8])
    {   
        data[0] = static_cast<uint8_t>(control_mode); // Control mode
        data[1] = static_cast<uint8_t>(move_mode); // MOVE mode
        data[2] = static_cast<uint8_t>(move_spd_rate_ctrl); // Speed percentage
        data[3] = static_cast<uint8_t>(mit_mode); // MIT mode
        // RCLCPP_INFO(rclcpp::get_logger("PiperArmMotionCtrl2"), "Write target for ID %03x: Control mode set to CAN command control mode, Move mode set to MOVE J, Speed set to 100%%", id);
        return true;
    }

    bool perform_switch(const std::vector<std::string> &start_interfaces,
                        const std::vector<std::string> &stop_interfaces)
    {
        return true;
    }
};
} // namespace can_data_plugins
