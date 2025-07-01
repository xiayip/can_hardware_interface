#pragma once

#include <map>
#include <utility>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include "can_data_plugins/can_data_base.hpp"

namespace can_data_plugins
{

struct PiperMotionCtrl2Data : public can_data_plugins::CanDataBase
{
    std::string                          joint_name;

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

    void write_target(const int id, uint8_t (&data)[8])
    {
    }

    bool perform_switch(const std::vector<std::string> &start_interfaces,
                        const std::vector<std::string> &stop_interfaces)
    {
        return true;
    }
};
} // namespace can_data_plugins