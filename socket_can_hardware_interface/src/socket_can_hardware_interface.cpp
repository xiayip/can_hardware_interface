#include "socket_can_hardware_interface/socket_can_hardware_interface.hpp"
#include <cstring>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <chrono>
#include <fcntl.h>

namespace socket_can_hardware_interface {

CallbackReturn SocketCanHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
{
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    can_socket_fd_ = -1;

    // setup node data
    for (size_t i = 0; i < info_.joints.size(); i++) {
        if (info_.joints[i].parameters["can_address"].empty()) {
            RCLCPP_ERROR(rclcpp::get_logger("SocketCanHardwareInterface"), "No CAN address for '%s'", info_.joints[i].name.c_str());
            return CallbackReturn::ERROR;
        }
        std::string device_type = info_.joints[i].parameters["device_type"];
        auto can_address = std::stoi(info_.joints[i].parameters["can_address"]);

        if (device_type == "PIPER") {
            std::map<int, std::string> canAdress_to_plugin = {
                {0x2A5, "can_data_plugins::PiperJointFeedback12"},
                {0x2A6, "can_data_plugins::PiperJointFeedback34"},
                {0x2A7, "can_data_plugins::PiperJointFeedback56"},
                {0x2A1, "can_data_plugins::PiperArmStatus"},
                {0x150, "can_data_plugins::PiperArmMotionCtrl1"},
                {0x151, "can_data_plugins::PiperArmMotionCtrl2"},
                {0x471, "can_data_plugins::PiperArmDisableEnableConfig"},
                {0x155, "can_data_plugins::PiperJointCtrl12"},
                {0x156, "can_data_plugins::PiperJointCtrl34"},
                {0x157, "can_data_plugins::PiperJointCtrl56"},
                {0x15A, "can_data_plugins::PiperJointMitCtrl1"},
                {0x15B, "can_data_plugins::PiperJointMitCtrl2"},
                {0x15C, "can_data_plugins::PiperJointMitCtrl3"},
                {0x15D, "can_data_plugins::PiperJointMitCtrl4"},
                {0x15E, "can_data_plugins::PiperJointMitCtrl5"},
                {0x15F, "can_data_plugins::PiperJointMitCtrl6"},
            };
            auto it = canAdress_to_plugin.find(can_address);
            if (it != canAdress_to_plugin.end()) {
                std::shared_ptr<can_data_plugins::CanDataBase> d = can_data_loader_.createSharedInstance(it->second);
                if (d->initialize(info_.joints[i])) {
                    can_data_[can_address] = d;
                }
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("SocketCanHardwareInterface"), "Unsupported CAN address '%u' for '%s'", can_address, info_.joints[i].name.c_str());
                return CallbackReturn::ERROR;
            }
        } else if (device_type == "OmniPicker") {
            std::shared_ptr<can_data_plugins::CanDataBase> omni_picker = can_data_loader_.createSharedInstance("can_data_plugins::OmniPickerData");
            if (omni_picker->initialize(info_.joints[i])) {
                can_data_[can_address] = omni_picker;
            }
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("SocketCanHardwareInterface"), "Unsupported device type '%s'", device_type.c_str());
            return CallbackReturn::ERROR;
        }
        RCLCPP_INFO(rclcpp::get_logger("SocketCanHardwareInterface"), "CAN address for '%s' is '%u'",
            info_.joints[i].name.c_str(), can_address);
    }
    return CallbackReturn::SUCCESS;
}

CallbackReturn SocketCanHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
    struct ifreq ifr;
    struct sockaddr_can addr;

    // Create a socket
    can_socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_fd_ < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("SocketCanHardwareInterface"), "Failed to create CAN socket");
        return CallbackReturn::ERROR;
    }

    // Specify the CAN interface
    std::string can_interface = info_.hardware_parameters["can_interface"];
    if (can_interface.empty()) {
        can_interface = "can0"; // Default interface
    }
    std::strncpy(ifr.ifr_name, can_interface.c_str(), IFNAMSIZ - 1);
    if (ioctl(can_socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("SocketCanHardwareInterface"), "Failed to get interface index for %s", can_interface.c_str());
        return CallbackReturn::ERROR;
    }

    // Bind the socket to the CAN interface
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_socket_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("SocketCanHardwareInterface"), "Failed to bind CAN socket to interface %s", can_interface.c_str());
        return CallbackReturn::ERROR;
    }
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 1000; // Reduce to 1 millisecond for faster response
    if (setsockopt(can_socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("SocketCanHardwareInterface"), "setsockopt failed");
        return CallbackReturn::ERROR;
    }
    
    // Set socket to non-blocking mode for better performance
    int flags = fcntl(can_socket_fd_, F_GETFL, 0);
    if (flags == -1) {
        RCLCPP_ERROR(rclcpp::get_logger("SocketCanHardwareInterface"), "Failed to get socket flags");
        return CallbackReturn::ERROR;
    }
    if (fcntl(can_socket_fd_, F_SETFL, flags | O_NONBLOCK) == -1) {
        RCLCPP_ERROR(rclcpp::get_logger("SocketCanHardwareInterface"), "Failed to set socket to non-blocking");
        return CallbackReturn::ERROR;
    }
    init_flag_ = false;
    can_bus_status_ = 0.0;
    RCLCPP_INFO(rclcpp::get_logger("SocketCanHardwareInterface"), "Successfully bound to CAN interface %s", can_interface.c_str());
    return CallbackReturn::SUCCESS;
}

CallbackReturn SocketCanHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    return CallbackReturn::SUCCESS;
}

CallbackReturn SocketCanHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    if (can_socket_fd_ >= 0) {
        close(can_socket_fd_);
        can_socket_fd_ = -1;
    }
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SocketCanHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    // Iterate over joints in xacro
    for (auto &[node_id, data] : can_data_) {
        data->export_state_interface(state_interfaces);
    }
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(info_.hardware_parameters["can_interface"], "status", &can_bus_status_));
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SocketCanHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    // Iterate over joints in xacro
    for (auto &[node_id, data] : can_data_) {
        data->export_command_interface(command_interfaces);
    }
    return command_interfaces;
}

hardware_interface::return_type
SocketCanHardwareInterface::perform_command_mode_switch(const std::vector<std::string> &start_interfaces,
                                                const std::vector<std::string> &stop_interfaces)
{
    // perform switching
    for (auto &[node_id, data] : can_data_) {
        if (!data->perform_switch(start_interfaces, stop_interfaces)) {
            return hardware_interface::return_type::ERROR;
        }
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type SocketCanHardwareInterface::read(const rclcpp::Time & /*time*/,
                                                         const rclcpp::Duration & /*period*/)
{
    
    if (!init_flag_) {
        return hardware_interface::return_type::OK;
    }

    struct can_frame frame;
    ssize_t nbytes;
    int frames_read = 0;
    const int max_frames_per_cycle = can_data_.size() * 4;
    const auto max_read_time = std::chrono::microseconds(can_data_.size() * 100); // Adjust based on expected frame size and processing time

    auto start_time = std::chrono::high_resolution_clock::now();
    while (frames_read <= max_frames_per_cycle) {
        auto current_time = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(current_time - start_time);
        
        // Break if we've spent too much time reading
        if (elapsed >= max_read_time) {
            if (frames_read > 0) {
                RCLCPP_WARN(rclcpp::get_logger("SocketCanHardwareInterface"), 
                           "Read timeout after %ld μs, processed %d frames", elapsed.count(), frames_read);
            }
            break;
        }
        
        nbytes = ::read(can_socket_fd_, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
            // Socket would block or no more data available
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                break; // No more data available, exit cleanly
            } else {
                RCLCPP_WARN(rclcpp::get_logger("SocketCanHardwareInterface"), "CAN read error: %s", strerror(errno));
                break;
            }
        } else if (nbytes < sizeof(struct can_frame)) {
            RCLCPP_ERROR(rclcpp::get_logger("SocketCanHardwareInterface"), "Incomplete CAN frame");
            return hardware_interface::return_type::ERROR;
        } else {
            frames_read++;
            // Process the received CAN frame
            auto it = can_data_.find(frame.can_id);
            if (it != can_data_.end()) {
                it->second->read_state(frame.can_id, frame.data);
            } else {
                // RCLCPP_WARN(rclcpp::get_logger("SocketCanHardwareInterface"),
                //             "Received frame with unknown CAN ID: 0x%X", frame.can_id);
            }
        }
    }
    
    if (frames_read > max_frames_per_cycle) {
        RCLCPP_DEBUG(rclcpp::get_logger("SocketCanHardwareInterface"), 
                   "Frames read: (%d) Hit max frames limit (%d) in read cycle, consider increasing cycle rate", frames_read, max_frames_per_cycle);
    }
    
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type SocketCanHardwareInterface::write(const rclcpp::Time & /*time*/,
                                                          const rclcpp::Duration & /*period*/)
{
    for (auto &[node_id, data] : can_data_) {
        struct can_frame frame;
        frame.can_id = node_id;
        frame.len = 8;
        std::memset(frame.data, 0, sizeof(frame.data));
        if (data->write_target(node_id, frame.data)) {
            ssize_t nbytes = ::write(can_socket_fd_, &frame, sizeof(struct can_frame));
            if (nbytes < 0) {
                RCLCPP_WARN(rclcpp::get_logger("SocketCanHardwareInterface"), "CAN write error for node ID 0x%02x", node_id);
                // return hardware_interface::return_type::ERROR;
            }
        }
    }
    init_flag_ = true;
    return hardware_interface::return_type::OK;
}

} // namespace socket_can_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(socket_can_hardware_interface::SocketCanHardwareInterface, hardware_interface::SystemInterface)
