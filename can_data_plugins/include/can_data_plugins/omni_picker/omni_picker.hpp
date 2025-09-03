#pragma once

#include <map>
#include <utility>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include "can_data_plugins/can_data_base.hpp"

namespace can_data_plugins
{

    struct OmniPickerData : public can_data_plugins::CanDataBase
    {
        enum ErrorCode
        {
            NoError = 0x00,
            Error_OverTemprature = 0x01,
            Error_OverVelocity = 0x02,
            Error_InitFailed = 0x03,
            Error_OverRun = 0x04,
        };

        enum State
        {
            PositionReached = 0x00,
            Moving = 0x01,
            Stalling = 0x02,
            ObejectFell = 0x03,
        };

        std::string joint_name;
        // FROM MOTOR
        uint8_t error_code = ErrorCode::NoError;
        uint8_t state = State::PositionReached;
        double actual_position = std::numeric_limits<double>::quiet_NaN();
        double actual_velocity = std::numeric_limits<double>::quiet_NaN();
        double actual_torque = std::numeric_limits<double>::quiet_NaN();
        // TO MOTOR
        double target_position = std::numeric_limits<double>::quiet_NaN();
        double target_velocity = std::numeric_limits<double>::quiet_NaN();
        double target_torque = std::numeric_limits<double>::quiet_NaN();
        double target_accleration = std::numeric_limits<double>::quiet_NaN();
        double target_deceleration = std::numeric_limits<double>::quiet_NaN();

        bool initialize(hardware_interface::ComponentInfo &joint)
        {
            joint_name = joint.name;
            target_position = 0.0;
            target_velocity = 1.0;
            target_torque = 1.0;
            target_accleration = 1.0;
            target_deceleration = 1.0;
            return true;
        }

        void export_state_interface(std::vector<hardware_interface::StateInterface> &state_interfaces)
        {
            state_interfaces.emplace_back(joint_name, hardware_interface::HW_IF_POSITION, &actual_position);
            state_interfaces.emplace_back(joint_name, hardware_interface::HW_IF_VELOCITY, &actual_velocity);
            state_interfaces.emplace_back(joint_name, hardware_interface::HW_IF_EFFORT, &actual_torque);
        }

        void export_command_interface(std::vector<hardware_interface::CommandInterface> &command_interfaces)
        {
            command_interfaces.emplace_back(joint_name, hardware_interface::HW_IF_POSITION, &target_position);
            command_interfaces.emplace_back(joint_name, hardware_interface::HW_IF_VELOCITY, &target_velocity);
            command_interfaces.emplace_back(joint_name, hardware_interface::HW_IF_EFFORT, &target_torque);
            // command_interfaces.emplace_back(joint_name, hardware_interface::HW_IF_ACCELERATION, &target_accleration);
            // command_interfaces.emplace_back(joint_name, "set_deceleration", &target_deceleration);
        }

        void read_state(const int id, const uint8_t data[8])
        {
            (void)id; // unused

            error_code = data[0];
            state = data[1];

            auto actual_position_raw = data[2];
            actual_position = static_cast<double>(actual_position_raw) / 255.0;
            auto actual_velocity_raw = data[3];
            actual_velocity = static_cast<double>(actual_velocity_raw) / 255.0;
            auto actual_torque_raw = data[4];
            actual_torque = static_cast<double>(actual_torque_raw) / 255.0;

            // debug
            // RCLCPP_INFO(rclcpp::get_logger("OmniPickerData"), "error_code: %d, state: %d, actual_position: %f, actual_velocity: %f, actual_torque: %f",
            // error_code, state, actual_position, actual_velocity, actual_torque);
        }

        bool write_target(const int id, uint8_t (&data)[8])
        {
            uint8_t target_position_raw = static_cast<uint8_t>((1.0f - target_position) * 255.0);
            data[1] = target_position_raw;
            uint8_t target_velocity_raw = static_cast<uint8_t>(target_velocity * 255.0);
            data[2] = target_velocity_raw;
            uint8_t target_torque_raw = static_cast<uint8_t>(target_torque * 255.0);
            data[3] = target_torque_raw;
            uint8_t target_accleration_raw = static_cast<uint8_t>(target_accleration * 255.0);
            data[4] = target_accleration_raw;
            uint8_t target_deceleration_raw = static_cast<uint8_t>(target_deceleration * 255.0);
            data[5] = target_deceleration_raw;

            // debug
            // RCLCPP_INFO(rclcpp::get_logger("OmniPickerData"), ">>>>>>>send cmd node id: %d, target_position: %f, target_velocity: %f, target_torque: %f, target_accleration: %f, target_deceleration: %f",
            //             id, target_position, target_velocity, target_torque, target_accleration, target_deceleration);
            // RCLCPP_INFO(rclcpp::get_logger("OmniPickerData"),
            //     ">>>>>>>send cmd data: %02x %02x %02x %02x %02x %02x %02x %02x ",
            //     (unsigned char)data[0], (unsigned char)data[1],
            //     (unsigned char)data[2], (unsigned char)data[3],
            //     (unsigned char)data[4], (unsigned char)data[5],
            //     (unsigned char)data[6], (unsigned char)data[7]);
            return true;
        }

        bool perform_switch(const std::vector<std::string> &start_interfaces,
                            const std::vector<std::string> &stop_interfaces)
        {
            (void)start_interfaces; // unused
            (void)stop_interfaces;  // unused
            return true;
        }
    };
} // namespace can_data_plugins