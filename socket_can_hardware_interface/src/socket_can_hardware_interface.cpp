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

namespace socket_can_hardware_interface {

CallbackReturn SocketCanHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
{
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    can_socket_fd_ = -1;

    // setup node data
    for (size_t i = 0; i < info_.joints.size(); i++) {
        if (info_.joints[i].parameters["node_id"].empty()) {
            RCLCPP_ERROR(rclcpp::get_logger("SocketCanHardwareInterface"), "No node id for '%s'", info_.joints[i].name.c_str());
            return CallbackReturn::ERROR;
        }
        std::string device_type = info_.joints[i].parameters["device_type"];
        auto node_id = std::stoi(info_.joints[i].parameters["node_id"]);

        if (device_type == "Piper") {
            node_id =  node_id;
            std::shared_ptr<can_data_plugins::CanDataBase> piper_data = can_data_loader_.createSharedInstance("can_data_plugins::LKMotorData");
            if (piper_data->initialize(info_.joints[i])) {
                can_data_[node_id] = piper_data;
            }
        }else {
            RCLCPP_ERROR(rclcpp::get_logger("SocketCanHardwareInterface"), "Unsupported motor type '%s'", device_type.c_str());
            return CallbackReturn::ERROR;
        }
        RCLCPP_INFO(rclcpp::get_logger("SocketCanHardwareInterface"), "Node id for '%s' is '%u'",
            info_.joints[i].name.c_str(), node_id);
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
    timeout.tv_usec = 100000; // 100 milliseconds
    if (setsockopt(can_socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
        perror("setsockopt failed");
        return CallbackReturn::ERROR;
    }
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
    struct can_frame frame;
    ssize_t nbytes;

    // Set the socket to non-blocking mode if desired
    // int flags = fcntl(can_socket_fd_, F_GETFL, 0);
    // fcntl(can_socket_fd_, F_SETFL, flags | O_NONBLOCK);

    while (true) {
        nbytes = ::read(can_socket_fd_, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // No more frames to read
                break;
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("SocketCanHardwareInterface"), "Error reading from CAN socket: %s", strerror(errno));
                return hardware_interface::return_type::ERROR;
            }
        } else if (nbytes < static_cast<ssize_t>(sizeof(struct can_frame))) { // fix the warning: 
            RCLCPP_ERROR(rclcpp::get_logger("SocketCanHardwareInterface"), "Incomplete CAN frame received");
            return hardware_interface::return_type::ERROR;
        } else {
            // Process the received CAN frame
            auto it = can_data_.find(frame.can_id);
            if (it != can_data_.end()) {
                it->second->read_state(frame.can_id, frame.data);
            } else {
                RCLCPP_WARN(rclcpp::get_logger("SocketCanHardwareInterface"),
                            "Received frame with unknown CAN ID: 0x%X", frame.can_id);
            }
        }
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
        data->write_target(node_id, frame.data);
        ssize_t nbytes = ::write(can_socket_fd_, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
            RCLCPP_WARN(rclcpp::get_logger("SocketCanHardwareInterface"), "CAN write error for node ID %u", node_id);
            return hardware_interface::return_type::ERROR;
        }
    }
    return hardware_interface::return_type::OK;
}

} // namespace socket_can_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(socket_can_hardware_interface::SocketCanHardwareInterface, hardware_interface::SystemInterface)
