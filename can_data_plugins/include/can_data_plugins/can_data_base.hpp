#pragma once

#include <vector>
#include <string>
#include <hardware_interface/system_interface.hpp>

namespace can_data_plugins {

class CanDataBase
{
public:
    virtual ~CanDataBase() {};

    virtual bool initialize(hardware_interface::ComponentInfo &joint) = 0;

    virtual void export_state_interface(std::vector<hardware_interface::StateInterface> &state_interfaces) = 0;

    virtual void export_command_interface(std::vector<hardware_interface::CommandInterface> &command_interfaces) = 0;

    virtual void read_state(const int id, const uint8_t data[8]) = 0;

    virtual void write_target(const int id, uint8_t (&data)[8]) = 0;

    virtual bool perform_switch(const std::vector<std::string> &start_interfaces,
                                const std::vector<std::string> &stop_interfaces) = 0;

protected:
    CanDataBase(){}
};
}