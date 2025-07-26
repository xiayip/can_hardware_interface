#pragma once

#include <map>
#include <utility>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include "can_data_plugins/can_data_base.hpp"

/*
    CAN ID:
        0x150

    Args:
        emergency_stop: Emergency stop command.
        track_ctrl: Trajectory control command.
        grag_teach_ctrl: Drag teach command.

    Bit Descriptions:

        Byte 0: emergency_stop: uint8, emergency stop control.
            0x00: Invalid.
            0x01: Activate emergency stop.
            0x02: Resume from emergency stop.

        Byte 1: track_ctrl: uint8, trajectory control instructions.
            0x00: Disable.
            0x01: Pause current planning.
            0x02: Resume current trajectory.
            0x03: Clear current trajectory.
            0x04: Clear all trajectories.
            0x05: Get the current planned trajectory.
            0x06: Terminate execution.
            0x07: Trajectory transfer.
            0x08: End trajectory transfer.

        Byte 2: grag_teach_ctrl: uint8, drag teach control.
            0x00: Disable.
            0x01: Start teaching record (enter drag teach mode).
            0x02: End teaching record (exit drag teach mode).
            0x03: Execute the teaching trajectory (reproduce drag teaching trajectory).
            0x04: Pause execution.
            0x05: Continue execution (resume trajectory reproduction).
            0x06: Terminate execution.
            0x07: Move to the starting point of the trajectory.

        Byte 3: trajectory_index: uint8, mark the transmitted trajectory point as the Nth trajectory point.
            N = 0~255:
            The controller responds with CAN ID: 0x476, Byte 0 = 0x50, Byte 2 = N. If no response is received, retransmission is required.

        Byte 4-5: NameIndex_H/L: uint16, current trajectory packet name index, composed of NameIndex and CRC.
            Response on CAN ID: 0x477, Byte 0 = 0x03.

        Byte 6-7: crc16_H/L: uint16, CRC checksum for validation.
*/

namespace can_data_plugins
{

struct PiperArmMotionCtrl1 : public can_data_plugins::CanDataBase
{

    bool init_flag_ = false;

    bool initialize(hardware_interface::ComponentInfo &joint)
    {
        init_flag_ = false;
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
        if (!init_flag_) {
            data[0] = 0x00; 
            data[1] = 0x00;
            data[2] = 0x00;
            // data[3] = 0x00; // Set MIT mode to Position-speed mode
            init_flag_ = true;
            // RCLCPP_INFO(rclcpp::get_logger("PiperArmMotionCtrl1"), "Write target for ID %03x: emergency_stop set to 0x00, track_ctrl set to 0x00, grag_teach_ctrl set to 0x00", id);
            return true;
        }
        return false;
    }

    bool perform_switch(const std::vector<std::string> &start_interfaces,
                        const std::vector<std::string> &stop_interfaces)
    {
        return true;
    }
};
} // namespace can_data_plugins
