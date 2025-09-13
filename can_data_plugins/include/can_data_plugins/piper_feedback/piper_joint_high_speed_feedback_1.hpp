#pragma once

#include <map>
#include <utility>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include "can_data_plugins/can_data_base.hpp"

/*
CAN ID:
    0x251~0x256

Args:
    can_id: Current CAN ID, used to represent the joint number.
    motor_speed: Motor Speed.
    current: Motor Current.
    pos: Motor Position.
    effort: Torque converted using a fixed coefficient, with a unit of 0.001 N/m.

Bit Description:

    Byte 0: Motor Speed (High Byte), int16, unit: 0.001rad/s
    Byte 1: Motor Speed (Low Byte)
    Byte 2: Motor Current (High Byte), uint16, unit: 0.001A
    Byte 3: Motor Current (Low Byte)
    Byte 4: Motor Position (Most Significant Byte), int32, unit: rad
    Byte 5: Motor Position (Second Most Significant Byte)
    Byte 6: Motor Position (Second Least Significant Byte)
    Byte 7: Motor Position (Least Significant Byte)
*/

namespace can_data_plugins
{

struct PiperJointHighSpeedFeedback1 : public can_data_plugins::CanDataBase
{
    std::string prefix;

    double COEFFICIENT_1_to_3 = 1.18125;
    double COEFFICIENT_4_to_6 = 0.95844;
    double joint_1_actual_velocity = 0.0;
    double joint_1_actual_effort = 0.0;

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
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(prefix + "joint1", hardware_interface::HW_IF_VELOCITY, &joint_1_actual_velocity));
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(prefix + "joint1", hardware_interface::HW_IF_EFFORT, &joint_1_actual_effort));
    }

    void export_command_interface(std::vector<hardware_interface::CommandInterface> &command_interfaces)
    {
    }

    void read_state(const int id, const uint8_t data[8])
    {
        // read joint 1 velocity from data[0] to data[1]
        int16_t joint_1_actual_velocity_raw = (int16_t(int8_t(data[0]) << 8) | uint8_t(data[1]));
        joint_1_actual_velocity = static_cast<double>(joint_1_actual_velocity_raw) / 1000.0; // Convert to rad/s
        // read joint 1 effort from data[2] to data[3]
        int16_t joint_1_actual_effort_raw = (int16_t(int8_t(data[2]) << 8) | uint8_t(data[3]));
        joint_1_actual_effort = static_cast<double>(joint_1_actual_effort_raw) * COEFFICIENT_1_to_3 / 1000.0; // Convert
        // RCLCPP_INFO(rclcpp::get_logger("PiperJointHighSpeedFeedback1"), "Read state for ID %d: Joint 1 Velocity: %.2f rad/s, Joint 1 Effort: %.2f N/m",
        //     id, joint_1_actual_velocity, joint_1_actual_effort);
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