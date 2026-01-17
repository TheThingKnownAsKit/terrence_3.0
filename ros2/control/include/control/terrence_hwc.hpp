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

namespace terrence_hwc
{
    class TerrenceHWC : public hardware_interface::SystemInterface
    {

        struct Config
        {
            std::string left_wheel_name = "";
            std::string right_wheel_name = "";
            float loop_rate = 0.0;
            std::string device = "";
            int baud_rate = 0;
            int timeout_ms = 0;
        }


        public:
            RCLCPP_SHARED_PTR_DEFINITIONS(TerrenceHWC)

            // Lifecycle
            hardware_interface::CallbackReturn on_init(
                const hardware_interface::HardwareInfo & info) override;
            
            hardware_interface::CallbackReturn on_configure(
                const rclcpp_lifecycle::State * previous_state) override;
            
            hardware_interface::CallbackReturn on_activate(
                const rclcpp_lifecycle::State & previous_state) override;
            
            hardware_interface::CallbackReturn on_deactivate(
                const rclcpp_lifecycle::State & previous_state) override;
            
            // rw
            hardware_interface::return_type read(
                const rclcpp::Time & time, const rclcpp::Duration & period) override;
            
            hardware_interface::return_type write(
                const rclcpp::Time & time, const rclcpp::Duration & period) override;
        
        private:
            Config config_;
    }
}   // namespace terrence_hwc


#endif // TERRENCE_HWC_HPP