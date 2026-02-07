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

        if (info_.hardware_parameters.count("max_radps") > 0) {
            config_.max_radps = std::stod(info_.hardware_parameters.at("max_radps"));
        } else {
            config_.max_radps = 10.0;
            RCLCPP_WARN(rclcpp::get_logger("TerrenceHWC"), "max_radps not specified in URDF, defaulting to 10.0");
        }

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

        // Debugging? I dont know her :)
        // for (const hardware_interface::ComponentInfo & joint : info_.joints)
        // {
        //     // DiffBotSystem has exactly two states and one command interface on each joint
        //     if (joint.command_interfaces.size() != 1)
        //     {
        //         RCLCPP_FATAL(
        //         rclcpp::get_logger("DiffDriveArduino"),
        //         "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        //         joint.command_interfaces.size());
        //         return hardware_interface::CallbackReturn::ERROR;
        //     }

        //     if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
        //     {
        //         RCLCPP_FATAL(
        //         rclcpp::get_logger("DiffDriveArduino"),
        //         "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        //         joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        //         return hardware_interface::CallbackReturn::ERROR;
        //     }
        // }

        left_vel_if_  = config_.left_wheel_name  + "/" + hardware_interface::HW_IF_VELOCITY;
        right_vel_if_ = config_.right_wheel_name + "/" + hardware_interface::HW_IF_VELOCITY;
        left_pos_if_  = config_.left_wheel_name  + "/" + hardware_interface::HW_IF_POSITION;
        right_pos_if_ = config_.right_wheel_name + "/" + hardware_interface::HW_IF_POSITION;

        loader_cmd_if_ = config_.loader_name + "/" + hardware_interface::HW_IF_POSITION;
        loader_pos_if_ = config_.loader_name + "/" + hardware_interface::HW_IF_POSITION;

        hopper_cmd_if_ = config_.hopper_name + "/" + hardware_interface::HW_IF_POSITION;
        hopper_pos_if_ = config_.hopper_name + "/" + hardware_interface::HW_IF_POSITION;

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

        left_pos_ = 0.0;
        right_pos_ = 0.0;
        left_vel_state_ = 0.0;
        right_vel_state_ = 0.0;
        loader_pos_ = 0.0;
        hopper_pos_ = 0.0;

        set_state(left_pos_if_, left_pos_);
        set_state(right_pos_if_, right_pos_);
        set_state(left_vel_if_, left_vel_state_);
        set_state(right_vel_if_, right_vel_state_);
        set_state(loader_pos_if_, loader_pos_);
        set_state(hopper_pos_if_, hopper_pos_);

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

        // safeguard against NaN
        const double dt = period.seconds();
        const double dt_safe = (std::isfinite(dt) && dt > 0.0 && dt < 0.5) ? dt : 0.0;

        // Open-loop: assume measured wheel velocity equals commanded wheel velocity
        const double left_cmd  = get_command<double>(left_vel_if_);
        const double right_cmd = get_command<double>(right_vel_if_);

        left_vel_state_  = left_cmd;
        right_vel_state_ = right_cmd;

        left_pos_  = finite_or_zero(left_pos_  + left_vel_state_  * dt_safe);
        right_pos_ = finite_or_zero(right_pos_ + right_vel_state_ * dt_safe);

        double l_cmd = get_command<double>(loader_cmd_if_);
        if (std::isfinite(l_cmd)) {
            loader_pos_ = l_cmd;
        }

        double h_cmd = get_command<double>(hopper_cmd_if_);
        if (std::isfinite(h_cmd)) {
            hopper_pos_ = h_cmd;
        }

        // Publish state into ros2_control-managed handles
        set_state(left_vel_if_, left_vel_state_);
        set_state(right_vel_if_, right_vel_state_);
        set_state(left_pos_if_, left_pos_);
        set_state(right_pos_if_, right_pos_);
        set_state(loader_pos_if_, loader_pos_);
        set_state(hopper_pos_if_, hopper_pos_);

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type TerrenceHWC::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        if (!comms_.connected())
        {
            return hardware_interface::return_type::ERROR;
        }

        double left_radps  = finite_or_zero(get_command<double>(left_vel_if_));
        double right_radps = finite_or_zero(get_command<double>(right_vel_if_));

        // Map Rad/s to RoboClaw 0-127 Scale
        // 0 = Full Rev, 64 = Stop, 127 = Full Fwd
        
        double max = config_.max_radps;
        
        // Calculate factor [-1.0 to 1.0]
        double left_factor = std::clamp(left_radps / max, -1.0, 1.0);
        double right_factor = std::clamp(right_radps / max, -1.0, 1.0);

        // Convert to 0-127 (where 64 is middle)
        // 64 + (factor * 63)
        int left_pwm = 64 + static_cast<int>(left_factor * 63.0);
        int right_pwm = 64 + static_cast<int>(right_factor * 63.0);

        // Safety Clamp
        left_pwm = std::clamp(left_pwm, 1, 127);
        right_pwm = std::clamp(right_pwm, 1, 127);

        comms_.set_motor_values(left_pwm, right_pwm);

        // TODO: Update arduinocomms to accept loader/hopper values
        // double loader_cmd = finite_or_zero(get_command<double>(loader_cmd_if_));
        // double hopper_cmd = finite_or_zero(get_command<double>(hopper_cmd_if_));

        return hardware_interface::return_type::OK;
    }

} // namespace terrence_hwc

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    terrence_hwc::TerrenceHWC,
    hardware_interface::SystemInterface
)