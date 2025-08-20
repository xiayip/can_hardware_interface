#pragma once

#include <cmath>
#include <utility>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include "can_data_plugins/can_data_base.hpp"

/*
    CAN ID:
        0x15E
    
    对应关节5的MIT控制
    
    Args:
        pos_ref: 设定期望的目标位置
        vel_ref: 设定电机运动的速度
        kp: 比例增益，控制位置误差对输出力矩的影响
        kd: 微分增益，控制速度误差对输出力矩的影响
        t_ref: 目标力矩参考值，用于控制电机施加的力矩或扭矩
        crc: 循环冗余校验，用于数据完整性验证
    
    位描述:
    
        Byte 0: Pos_ref [bit15~bit8] 高8位
        Byte 1: Pos_ref [bit7~bit0]  低8位
        Byte 2: Vel_ref [bit11~bit4] 低12位
        Byte 3: Vel_ref [bit3~bit0], Kp [bit11~bit8]
        Byte 4: Kp [bit7~bit0],      Kp给定参考值: 10
        Byte 5: Kd [bit11~bit4]      低12位,Kd给定参考值: 0.8
        Byte 6: Kd [bit3~bit0] T_ref [bit7~bit4]
        Byte 7: T_ref [bit3~bit0]
*/

namespace can_data_plugins
{

struct PiperJointMitCtrl5 : public can_data_plugins::CanDataBase
{
    double joint_5_target_position = std::numeric_limits<double>::quiet_NaN();
    double joint_5_target_velocity = std::numeric_limits<double>::quiet_NaN();
    double joint_5_target_kp = std::numeric_limits<double>::quiet_NaN();
    double joint_5_target_kd = std::numeric_limits<double>::quiet_NaN();
    double joint_5_target_torque = std::numeric_limits<double>::quiet_NaN();

    bool initialize(hardware_interface::ComponentInfo &joint)
    {
        joint_5_target_velocity = 0.0;
        joint_5_target_kp = 10.0;
        joint_5_target_kd = 0.8;
        joint_5_target_torque = 0.0;
        return true;
    }

    void export_state_interface(std::vector<hardware_interface::StateInterface> &state_interfaces)
    {
    }

    void export_command_interface(std::vector<hardware_interface::CommandInterface> &command_interfaces)
    {
        RCLCPP_INFO(rclcpp::get_logger("PiperJointMitCtrl5"), "Exporting joint 5 command interfaces");
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface("joint5", hardware_interface::HW_IF_POSITION, &joint_5_target_position));
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface("joint5", hardware_interface::HW_IF_VELOCITY, &joint_5_target_velocity));
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface("joint5", hardware_interface::HW_IF_EFFORT, &joint_5_target_torque));
    }

    void read_state(const int id, const uint8_t data[8])
    {
    }

    bool write_target(const int id, uint8_t (&data)[8])
    {
        if (std::isnan(joint_5_target_position)) {
            return false;
        }
        unsigned int pos_tmp = FloatToUint(joint_5_target_position, -12.5, 12.5, 16); // TODO: read from joint parameters
        unsigned int vel_tmp = FloatToUint(joint_5_target_velocity, -45.0, 45.0, 12);
        unsigned int kp_tmp = FloatToUint(joint_5_target_kp, 0.0, 500.0, 12);
        unsigned int kd_tmp = FloatToUint(joint_5_target_kd, -5.0, 5.0, 12);
        unsigned int t_tmp = FloatToUint(joint_5_target_torque, -8.0, 8.0, 8);

        data[0] = (pos_tmp >> 8) & 0xFF; // High byte of position
        data[1] = pos_tmp & 0xFF; // Low byte of position
        data[2] = (vel_tmp >> 4) & 0xFF; // High byte of velocity
        data[3] = ((vel_tmp & 0x0F) << 4) | ((kp_tmp >> 8) & 0x0F); // Low byte of velocity and high nibble of kp
        data[4] = kp_tmp & 0xFF; // Low byte of kp
        data[5] = (kd_tmp >> 4) & 0xFF; // High byte of kd
        data[6] = ((kd_tmp & 0x0F) << 4) | ((t_tmp >> 4) & 0x0F); // Low byte of kd and high nibble of torque
        data[7] = (t_tmp & 0x0F) | 0x00; // Low byte of torque and CRC placeholder

        RCLCPP_INFO(rclcpp::get_logger("PiperJointMitCtrl5"), "Write target for ID %03x: Joint 5 pos: %.2f, vel: %.2f, kp: %.2f, kd: %.2f, t: %.2f",
            id, joint_5_target_position, joint_5_target_velocity, joint_5_target_kp, joint_5_target_kd, joint_5_target_torque);
        return true;
    }

    bool perform_switch(const std::vector<std::string> &start_interfaces,
                        const std::vector<std::string> &stop_interfaces)
    {
        return true;
    }

    unsigned int FloatToUint(float x_float, float x_min, float x_max, int bits) {
        float span = x_max - x_min;
        float offset = x_min;
        
        // Calculate the maximum value for the given number of bits
        unsigned int max_value = (1U << bits) - 1;
        
        // Perform the linear mapping and conversion
        float normalized = (x_float - offset) / span;
        unsigned int result = static_cast<unsigned int>(normalized * max_value);
        
        return result;
    }
};
} // namespace can_data_plugins
