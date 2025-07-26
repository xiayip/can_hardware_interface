#pragma once

#include <map>
#include <utility>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include "can_data_plugins/can_data_base.hpp"

namespace can_data_plugins
{

struct PiperArmDisableEnableConfig : public can_data_plugins::CanDataBase
{
    bool init_flag_ = false;

    bool initialize(hardware_interface::ComponentInfo &joint)
    {
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
            data[0] = 0xFF; // select all joints
            data[1] = 0x02; // enable arm
            init_flag_ = true;
            // RCLCPP_INFO(rclcpp::get_logger("PiperArmDisableEnableConfig"), "Write target for ID %03x: Arm enabled", id);
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
