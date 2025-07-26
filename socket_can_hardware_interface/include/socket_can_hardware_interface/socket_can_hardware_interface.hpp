#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <pluginlib/class_loader.hpp>

#include "can_data_plugins/can_data_base.hpp"

namespace socket_can_hardware_interface {
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class SocketCanHardwareInterface : public hardware_interface::SystemInterface
{
public:
    // Initialize the hardware
    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    // Read the current state of the hardware (e.g., sensor data)
    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    // Write commands to actuators
    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    hardware_interface::return_type
    perform_command_mode_switch(const std::vector<std::string> &start_interfaces,
                                const std::vector<std::string> &stop_interfaces) override;

    // Required by ros2_control
    std::vector<hardware_interface::StateInterface>   export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

private:
    bool init_flag_;

    int can_socket_fd_;
    double can_bus_status_;

    // can stuff
    pluginlib::ClassLoader<can_data_plugins::CanDataBase> can_data_loader_{"can_data_plugins", "can_data_plugins::CanDataBase"};
    std::map<uint32_t, std::shared_ptr<can_data_plugins::CanDataBase>> can_data_;
};
} // namespace socket_can_hardware_interface