#ifndef TERRENCE_HWC_HPP
#define TERRENCE_HWC_HPP

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "control/arduino_comms.hpp"

namespace terrence_hwc
{
    class TerrenceHWC : public hardware_interface::SystemInterface
    {

        struct Config
        {
            std::string left_wheel_name = "";
            std::string right_wheel_name = "";
            std::string loader_name = "LoaderJoint";
            std::string hopper_name = "HopperJoint";

            float loop_rate = 0.0;
            std::string device = "";
            int baud_rate = 0;
            int timeout_ms = 0;
            double max_radps = 0.0;
            int pid_p = 0;
            int pid_d = 0;
            int pid_i = 0;
            int pid_o = 0;
        };

        public:
            RCLCPP_SHARED_PTR_DEFINITIONS(TerrenceHWC)

            // Lifecycle
            hardware_interface::CallbackReturn on_init(
                const hardware_interface::HardwareComponentInterfaceParams & params) override;
            
            hardware_interface::CallbackReturn on_configure(
                const rclcpp_lifecycle::State & previous_state) override;
            
            hardware_interface::CallbackReturn on_cleanup(
                const rclcpp_lifecycle::State & previous_state) override;
            
            hardware_interface::CallbackReturn on_activate(
                const rclcpp_lifecycle::State & previous_state) override;
            
            hardware_interface::CallbackReturn on_deactivate(
                const rclcpp_lifecycle::State & previous_state) override;
            
            // rw
            hardware_interface::return_type read(
                const rclcpp::Time & time, const rclcpp::Duration & period) override;
            
            hardware_interface::return_type write(
                const rclcpp::Time & time, const rclcpp::Duration & period) override;
            
            // util
            static inline double finite_or_zero(double x) {
                // checks to see if a double is finite, if it isn't, set to 0 to guard against NaN
                return std::isfinite(x) ? x : 0.0;
            }
        
        private:
            ArduinoComms comms_;
            Config config_;
            std::string left_vel_if_;
            std::string right_vel_if_;
            std::string left_pos_if_;
            std::string right_pos_if_;
            std::string loader_cmd_if_;
            std::string loader_pos_if_;
            std::string hopper_cmd_if_;
            std::string hopper_pos_if_;

            double left_pos_{0.0};
            double right_pos_{0.0};
            double left_vel_state_{0.0};
            double right_vel_state_{0.0};
            double loader_pos_{0.0};
            double hopper_pos_{0.0};
    };
}   // namespace terrence_hwc


#endif // TERRENCE_HWC_HPP