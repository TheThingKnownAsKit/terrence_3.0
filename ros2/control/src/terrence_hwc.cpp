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

        config_.left_wheel_name  = info_.hardware_parameters.at("left_wheel_name");
        config_.right_wheel_name = info_.hardware_parameters.at("right_wheel_name");
        config_.device           = info_.hardware_parameters.at("device");

        config_.loop_rate  = hardware_interface::stod(info_.hardware_parameters.at("loop_rate"));
        config_.baud_rate  = std::stoi(info_.hardware_parameters.at("baud_rate"));
        config_.timeout_ms = std::stoi(info_.hardware_parameters.at("timeout_ms"));

        if (info_.hardware_parameters.count("pid_p") > 0)
        {
            config_.pid_p = std::stoi(info_.hardware_parameters.at("pid_p"));
            config_.pid_i = std::stoi(info_.hardware_parameters.at("pid_i"));
            config_.pid_d = std::stoi(info_.hardware_parameters.at("pid_d"));
            config_.pid_o = std::stoi(info_.hardware_parameters.at("pid_o"));
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

        left_vel_if_  = config_.left_wheel_name  + "/" + hardware_interface::HW_IF_VELOCITY;
        right_vel_if_ = config_.right_wheel_name + "/" + hardware_interface::HW_IF_VELOCITY;
        left_pos_if_  = config_.left_wheel_name  + "/" + hardware_interface::HW_IF_POSITION;
        right_pos_if_ = config_.right_wheel_name + "/" + hardware_interface::HW_IF_POSITION;

        return hardware_interface::CallbackReturn::SUCCESS;
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

        left_pos_ = right_pos_ = 0.0;
        left_vel_state_ = right_vel_state_ = 0.0;

        set_state(left_pos_if_, left_pos_);
        set_state(right_pos_if_, right_pos_);
        set_state(left_vel_if_, left_vel_state_);
        set_state(right_vel_if_, right_vel_state_);

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

        // NOTE: Open loop odometry since no encoders

        // Open-loop: assume measured wheel velocity equals commanded wheel velocity
        const double left_cmd  = get_command<double>(left_vel_if_);
        const double right_cmd = get_command<double>(right_vel_if_);

        left_vel_state_  = left_cmd;
        right_vel_state_ = right_cmd;

        const double dt = period.seconds();
        if (dt > 0.0 && std::isfinite(dt))
        {
            left_pos_  += left_vel_state_  * dt;
            right_pos_ += right_vel_state_ * dt;
        }

        // Publish state into ros2_control-managed handles
        set_state(left_vel_if_, left_vel_state_);
        set_state(right_vel_if_, right_vel_state_);
        set_state(left_pos_if_, left_pos_);
        set_state(right_pos_if_, right_pos_);

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type TerrenceHWC::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        if (!comms_.connected())
        {
            return hardware_interface::return_type::ERROR;
        }

        const double left_cmd  = get_command<double>(left_vel_if_);
        const double right_cmd = get_command<double>(right_vel_if_);

        comms_.set_motor_values(left_cmd, right_cmd);

        return hardware_interface::return_type::OK;
    }

} // namespace terrence_hwc

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    terrence_hwc::TerrenceHWC,
    hardware_interface::SystemInterface
)