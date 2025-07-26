#pragma once

#include <map>
#include <utility>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include "can_data_plugins/can_data_base.hpp"

/*
    CAN ID: 
        0x2A1

    Arguments:
        ctrl_mode: Control mode
        arm_status: Robot arm status
        mode_feed: Mode feedback
        teach_status: Teaching status
        motion_status: Motion status
        trajectory_num: Current trajectory point number
        err_code: Error code
    
    Bit Description:

        Byte 0: Control mode, uint8
            0x00: Standby mode
            0x01: CAN instruction control mode
            0x02: Teaching mode
            0x03: Ethernet control mode
            0x04: Wi-Fi control mode
            0x05: Remote control mode
            0x06: Linkage teaching input mode
            0x07: Offline trajectory mode
        Byte 1: Robot arm status, uint8
            0x00: Normal
            0x01: Emergency stop
            0x02: No solution
            0x03: Singularity point
            0x04: Target angle exceeds limit
            0x05: Joint communication exception
            0x06: Joint brake not released
            0x07: Collision occurred
            0x08: Overspeed during teaching drag
            0x09: Joint status abnormal
            0x0A: Other exception
            0x0B: Teaching record
            0x0C: Teaching execution
            0x0D: Teaching pause
            0x0E: Main controller NTC over temperature
            0x0F: Release resistor NTC over temperature
        Byte 2: Mode feedback, uint8
            0x00: MOVE P
            0x01: MOVE J
            0x02: MOVE L
            0x03: MOVE C
            0x04: MOVE M
        Byte 3: Teaching status, uint8
            0x00: Off
            0x01: Start teaching record (enter drag teaching mode)
            0x02: End teaching record (exit drag teaching mode)
            0x03: Execute teaching trajectory (reproduce drag teaching trajectory)
            0x04: Pause execution
            0x05: Continue execution (continue trajectory reproduction)
            0x06: Terminate execution
            0x07: Move to trajectory starting point
        Byte 4: Motion status, uint8
            0x00: Reached the target position
            0x01: Not yet reached the target position
        Byte 5: Current trajectory point number, uint8_t
            0~255 (feedback in offline trajectory mode)
        Byte 6: Error code, uint16
            bit[0]: Joint 1 angle limit exceeded (0: normal, 1: abnormal)
            bit[1]: Joint 2 angle limit exceeded (0: normal, 1: abnormal)
            bit[2]: Joint 3 angle limit exceeded (0: normal, 1: abnormal)
            bit[3]: Joint 4 angle limit exceeded (0: normal, 1: abnormal)
            bit[4]: Joint 5 angle limit exceeded (0: normal, 1: abnormal)
            bit[5]: Joint 6 angle limit exceeded (0: normal, 1: abnormal)
            bit[6]: Reserved
            bit[7]: Reserved
        Byte 7: Error code, uint16
            bit[0]: Joint 1 communication exception (0: normal, 1: abnormal)
            bit[1]: Joint 2 communication exception (0: normal, 1: abnormal)
            bit[2]: Joint 3 communication exception (0: normal, 1: abnormal)
            bit[3]: Joint 4 communication exception (0: normal, 1: abnormal)
            bit[4]: Joint 5 communication exception (0: normal, 1: abnormal)
            bit[5]: Joint 6 communication exception (0: normal, 1: abnormal)
            bit[6]: Reserved
            bit[7]: Reserved
*/

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
        uint8_t limit_err_code = data[6];
        uint8_t comm_err_code = data[7];
        if (limit_err_code != 0) {
            int joint_index = __builtin_ffs(limit_err_code) - 1;
            RCLCPP_ERROR(rclcpp::get_logger("PiperArmStatus"), "Limit error joint_index: %d", joint_index);
        }
        if (comm_err_code != 0) {
            int joint_index = __builtin_ffs(comm_err_code) - 1;
            RCLCPP_ERROR(rclcpp::get_logger("PiperArmStatus"), "Comm error joint_index: %d", joint_index);
        }
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
