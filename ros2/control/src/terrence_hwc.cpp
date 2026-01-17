#include "control/terrence_hwc.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace terrence_hwc {

    hardware_interface::CallbackReturn TerrenceHWC::on_init(
        const hardware_interface::HardwareComponentInterfaceParams & params)
    {
        if (
            hardware_interface::SystemInterface::on_init(params) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        config_.left_wheel_name = stod(info_.hardware_parameters["left_wheel_name"]);
        config_.right_wheel_name = stod(info_.hardware_parameters["right_wheel_name"]);
        config_.loop_rate = stod(info_.hardware_parameters["loop_rate"]);
        config_.device = stod(info_.hardware_parameters["device"]);
        config_.baud_rate = stod(info_.hardware_parameters["baud_rate"]);
        config_.timeout_ms = stod(info_.hardware_parameters["timeout_ms"]);
        if (info_.hardware_parameters.count("pid_p") > 0)
        {
            config_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
            config_.pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
            config_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
            config_.pid_o = std::stoi(info_.hardware_parameters["pid_o"]);
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("TerrenceHWC"), "PID values not supplied, using defaults.");
        }

        for (const hardware_interface::ComponentInfo & joint : info_.joints)
        {
            // DiffBotSystem has exactly two states and one command interface on each joint
            if (joint.command_interfaces.size() != 1)
            {
                RCLCPP_FATAL(
                rclcpp::get_logger("DiffDriveArduino"),
                "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                rclcpp::get_logger("DiffDriveArduino"),
                "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // We should not have any state interfaces

    std::vector<hardware_interface::CommandInterface> TerrenceHWC::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            config_.left_wheel_name, hardware_interface::HW_IF_VELOCITY, &commands_.left_wheel_cmd
        ));

        command_interfaces.emplace_back(hardware_interfaces::CommandInterface(
            config_.right_wheel_name. hardware_interface::HW_IF_VELOCITY, &commands_.right_wheel_cmd
        ));

        return command_interfaces;
    }

    hardware_interface::CallbackReturn TerrenceHWC::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("TerrenceHWC"), "Configuring ... please wait ...");
        if (comms_.connected())
        {
            comms_.disconnect();
        }
        comms_.connect(config_.device, config_.baud_rate, config_.timeout_ms);
        RCLCPP_INFO(rclcpp::get_logger("TerrenceHWC"), "Successfully configured!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn TerrenceHWC::on_cleanup(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("TerrenceHWC"), "Cleaning up ... please wait ...");
        if (comms_.connected())
        {
            comms_.disconnect();
        }
        RCLCPP_INFO(rclcpp::get_logger("TerrenceHWC"), "Successfully cleaned up!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn TerrenceHWC::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("TerrenceHWC"), "Activating ... please wait ...");
        if (!comms_.connected())
        {
            return hardware_interface::CallbackReturn::ERROR;
        }
        RCLCPP_INFO(rclcpp::get_logger("TerrenceHWC"), "Successfully activated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn TerrenceHWC::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("TerrenceHWC"), "Deactivating ... please wait ...");
        RCLCPP_INFO(rclcpp::get_logger("TerrenceHWC"), "Successfully deactivated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type TerrenceHWC::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
    {
        if (!comms_.connected())
        {
            return hardware_interface::return_type::ERROR;
        }

        // we read nothing from serial

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type terrence_hwc ::TerrenceHWC::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        if (!comms_.connected())
        {
            return hardware_interface::return_type::ERROR;
        }

        comms_.set_motor_values(commands_.left_wheel_cmd, commands_.right_wheel_cmd)

        return hardware_interface::return_type::OK;
    }

} // namespace terrence_hwc

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    terrence_hwc::TerrenceHWC,
    hardware_interface::SystemInterface
)