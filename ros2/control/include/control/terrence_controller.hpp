#ifndef TERRENCE_CONTROLLER_HPP
#define TERRENCE_CONTROLLER_HPP

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <string>
#include <cmath>
#include <array>
#include <cstddef>
#include <cstdint>

namespace terrence_controller
{
    class TerrenceController : public controller_interface::ControllerInterface
    {
        public:
            TerrenceController() = default;

            controller_interface::CallbackReturn on_init() override;

            controller_interface::InterfaceConfiguration command_interface_configuration() const override;
            controller_interface::InterfaceConfiguration state_interface_configuration() const override;

            controller_interface::CallbackReturn on_configure(
                const rclcpp_lifecycle::State & previous_state) override;
            
            controller_interface::CallbackReturn on_activate(
                const rclcpp_lifecycle::State & previous_state) override;
            
            controller_interface::CallbackReturn on_deactivate(
                const rclcpp_lifecycle::State & previous_state) override;
            
            controller_interface::return_type update(
                const rclcpp::Time & time, const rclcpp::Duration & period) override;
            
        private:
            enum class Mode : uint8_t
            {
                IDLE = 0,
                DRIVE = 1,
                DIG = 2,
                DUMP = 3,
                FAULT = 4
            };

            static std::string modeToString(Mode m);
            static bool parseModeString(const std::string & s, Mode & out);

            // Realtime command buffers
            struct CmdVel
            {
                double linear_x{0.0};
                double angular_z{0.0};
                rclcpp::Time stamp{};
                bool valid{false};
            };

            struct DigCmd
            {
                // [0]=loader_target_position_rad, [1]=hopper_target_position_rad
                std::array<double, 8> data{};
                size_t len{0};
                rclcpp::Time stamp{};
                bool valid{false};
            };

            struct ModeRequest
            {
                Mode requested{Mode::IDLE};
                rclcpp::Time stamp{};
                bool valid{false};
            };

            realtime_tools::RealtimeBuffer<CmdVel> rt_cmd_vel_;
            realtime_tools::RealtimeBuffer<DigCmd> rt_dig_cmd_;
            realtime_tools::RealtimeBuffer<ModeRequest> rt_mode_req_;

            // ROS subscriptions
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
            rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr dig_cmd_sub_;
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr set_mode_sub_;
            rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_fault_srv_;

            // Params
            std::string left_joint_name_{"DS_Joint"};
            std::string right_joint_name_{"PS_Joint"};

            double wheel_radius_m_{0.085};
            double wheel_separation_m_{0.42};

            double max_wheel_radps_{10.0};
            double cmd_timeout_s_{0.25};
            double moving_eps_radps_{0.05};       // threshold to consider "moving"

            // State
            Mode mode_{Mode::IDLE};
            bool fault_latched_{false};

            // Cached indices for update()
            int left_cmd_idx_{-1};
            int right_cmd_idx_{-1};

            int left_pos_state_idx_{-1};
            int left_vel_state_idx_{-1};
            int right_pos_state_idx_{-1};
            int right_vel_state_idx_{-1};

            // Helper functions (cb = callback)
            // Keep as much processing out of update as possible by giving it to other functions
            void cmdVelCb(const geometry_msgs::msg::Twist & msg);
            void digCmdCb(const std_msgs::msg::Float64MultiArray & msg);
            void setModeCb(const std_msgs::msg::String & msg);

            void latchFault(const std::string & reason);
            void clearFault();

            bool isMovingNow() const;
            bool canTransition(Mode from, Mode to, bool moving_now) const;
            void enterMode(Mode new_mode);

            void setWheelCommandsRadps(double left_radps, double right_radps);

            // Accepts Twist 
            void computeWheelRadps(double v_mps, double w_radps,
                                double & out_left, double & out_right) const;
            
            // Maybe put debug/status publishing here later
    };
} // namespace terrence_controller

#endif // TERRENCE_CONTROLLER_HPP