#include "control/terrence_controller.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace terrence_controller
{
    std::string TerrenceController::modeToString(Mode m)
    {
        switch (m)
        {
            case Mode::IDLE: return "IDLE";
            case Mode::DRIVE: return "DRIVE";
            case Mode::DIG: return "DIG";
            case Mode::DUMP: return "DUMP";
            case Mode::FAULT: return "FAULT";
            default: return "UNKNOWN";
        }
    }

    bool TerrenceController::parseModeString(const std::string & s, Mode & out)
    {
        // for safety, uppercase the whole string
        auto up = s;
        for (auto & c : up) c = static_cast<char>(::toupper(c));

        if (up == "IDLE") { out = Mode::IDLE; return true; }
        if (up == "DRIVE") { out = Mode::DRIVE; return true; }
        if (up == "DIG") { out = Mode::DIG; return true; }
        if (up == "DUMP") { out = Mode::DUMP; return true; }
        if (up == "FAULT") { out = Mode::FAULT; return true; }
        return false;
    }

    controller_interface::CallbackReturn TerrenceController::on_init()
    {
        try
        {
            auto_declare<std::string>("left_joint_name", "DS_Joint");
            auto_declare<std::string>("right_joint_name", "PS_JOINT");

            auto_declare<double>("wheel_radius_m", 0.085);
            auto_declare<double>("wheel_separation_m", 0.42);

            auto_declare<double>("max_wheel_radps", 10.0);
            auto_declare<double>("cmd_timeout_s", 0.25);
            auto_declare<double>("moving_eps_radps", 0.05);
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "on_init exception: %s", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration TerrenceController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_inface::interface_configuration_type::INDIVIDUAL;
        config.names = {
            left_joint_name_ + "/velocity",
            right_joint_name_ + "/velocity"
        };

        return config;
    }

    controller_interface::InterfaceConfiguration TerrenceController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        config.names = {
            left_joint_name_ + "/position",
            left_joint_name_ + "/velocity",
            right_joint_name_ + "/position",
            right_joint_name_ + "/velocity"
        };

        return config;
    }

    controller_interface::CallbackReturn TerrenceController::on_configure(const rclcpp_lifecycle::State &)
    {
        left_joint_name_ = get_node()->get_parameter("left_joint_name").as_string();
        right_joint_name_ = get_node()->get_parameter("right_joint_name").as_string();

        wheel_radius_m_ = get_node()->get_parameter("wheel_radius_m").as_double();
        wheel_separation_m_ = get_node()->get_parameter("wheel_separation_m").as_double();

        max_wheel_radps_ = get_node()->get_parameter("max_wheel_radps").as_double();
        cmd_timeout_s_ = get_node()->get_parameter("cmd_timeout_s").as_double();
        moving_eps_radps_ = get_node()->get_parameter("moving_eps_radps").as_double();

        // Initialize the realtime buffers
        rt_cmd_vel_.writeFromNonRT(CmdVel{0.0, 0.0, get_node()->now(), false});
        rt_dig_cmd_.writeFromNonRT(DigCmd{});
        rt_mode_req_.writeFromNonRT(ModeRequest{Mode::IDLE, get_node()->now(), false});

        // ROS subscriptions
        cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", rclcpp::SystemDefaultsQoS(),
            [this](const geometry_msgs::msg::Twist & msg) { cmdVelCb(msg); });

        dig_cmd_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
            "/dig_cmd", rclcpp::SystemDefaultsQoS(),
            [this](const std_msgs::msg::Float64MultiArray & msg) { digCmdCb(msg); });
        
        // This could be a service but it's easier as a topic rn
        set_mode_sub_ = get_node()->create_subscription<std_msgs::msg::String>(
            "/set_mode", rclcpp::SystemDefaultsQoS(),
            [this](const std_msgs::msg::String & msg) { setModeCb(msg); });

        reset_fault_srv_ = get_node()->create_service<std_srvs::srv::Trigger>(
            "/resest_fault",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
            {
                clearFault();
                resp->success = true;
                resp->message = "Fault cleared; mode set to IDLE.";
            });
        
        mode_ = Mode::IDLE;
        fault_latched_ = false;

        RCLCPP_INFO(get_node()->get_logger(),
              "Configured TerrenceController. Initial mode=%s",
              modeToString(mode_).c_str());
        
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn TerrenceController::on_activate(const rclcpp_lifecycle::State &)
    {
        // Cache interface indices
        left_cmd_idx_ = right_cmd_idx_ = -1;
        left_pos_state_idx_ = left_vel_state_idx_ = -1;
        right_pos_state_idx_ = right_vel_state_idx_ = -1;

        for (size_t i = 0; i < command_interfaces_.size(); ++i)
        {
            const auto & ci = command_interfaces_[i];
            if (ci.get_name() == left_joint_name_ && ci.get_interface_name() == "velocity")
                left_cmd_idx_ = static_cast<int>(i);
            if (ci.get_name() == right_joint_name_ && ci.get_interface_name() == "velocity")
                right_cmd_idx_ = static_cast<int>(i);
        }

        for (size_t i = 0; i < state_interfaces_.size(); ++i)
        {
            const auto & si = state_interfaces_[i];
            const auto & jn = si.get_name(); // Joint name
            const auto & in = si.get_interface_name();

            if (jn == left_joint_name_ && in == "position") left_pos_state_idx_ = static_cast<int>(i);
            if (jn == left_joint_name_ && in == "velocity") left_vel_state_idx_ = static_cast<int>(i);
            if (jn == right_joint_name_ && in == "position") right_pos_state_idx_ = static_cast<int>(i);
            if (jn == right_joint_name_ && in == "velocity") right_vel_state_idx_ = static_cast<int>(i);
        }

        if (left_cmd_idx_ < 0 || right_cmd_idx_ < 0)
        {
            RCLCPP_ERROR(get_node()->get_logger(),
                 "Missing command interfaces. Need %s/velocity and %s/velocity.",
                 left_joint_name_.c_str(), right_joint_name_.c_str());
            return controller_interface::CallbackReturn::ERROR;
        }

        if (left_pos_state_idx_ < 0 || left_vel_state_idx_ < 0 ||
            right_pos_state_idx_ < 0 || right_vel_state_idx_ < 0)
        {
            RCLCPP_WARN(get_node()->get_logger(),
                        "Some state interfaces are missing. Moving detection/odom may be degraded.");
        }

        // Safety outputs
        setWheelCommandsRadps(0.0, 0.0);
        enterMode(Mode::IDLE);

        RCLCPP_INFO(get_node()->get_logger(), "Activated TerrenceController.");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn TerrenceController::on_deactivate(const rclcpp_lifecycle::State &)
    {
        // Stop outputs
        setWheelCommandsRadps(0.0, 0.0);
        RCLCPP_INFO(get_node()->get_logger(), "Deactivated TerrenceController.");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    // All of the callbacks

    void TerrenceController::cmdVelCb(const geometry_msgs::msg::Twist & msg)
    {
        CmdVel c;
        c.linear_x = msg.linear.x;
        c.angular_z = msg.angular.z;
        c.stamp = get_node()->now();
        c.valid = true;
        rt_cmd_vel_.writeFromNonRT(c);
    }

    void TerrenceController::digCmdCb(const std_msgs::msg::Float64MultiArray & msg)
    {
        DigCmd d;
        d.stamp = get_node()->now();
        d.valid = true;

        d.len = std::min<size_t>(msg.data.size(), d.data.size());
        for (size_t i = 0; i < d.len; ++i)
            d.data[i] = msg.data[i];

        rt_dig_cmd_.writeFromNonRT(d);
    }

    void TerrenceController::setModeCb(const std_msgs::String & msg)
    {
        Mode m;
        if (!parseModeString(msg.data, m))
        {
            RCLCPP_WARN(get_node()->get_logger(),
                        "Invalid /set_mode value '%s'. Use IDLE, DRIVE, DIG, DUMP, FAULT.",
                        msg.data.c_str());
            return;
        }

        ModeRequest r;
        r.requested = m;
        r.stamp = get_node()->now();
        r.valid = true;
        rt_mode_req_.writeFromNonRT(r);
    }

    void TerrenceController::latchFault(const std::string & reason)
    {
        if (!fault_latched_)
        {
            fault_latched_ = true;
            mode_ = Mode::FAULT;
            RCLCPP_ERROR(get_node()->get_logger(), "FAULT latched: %s", reason.c_str());
        }
    }

    void TerrenceController::clearFault()
    {
        fault_latched_ = false;
        mode_ = Mode::IDLE;
        // Clear pending requests to avoid immediate re-trigger
        rt_mode_req_.writeFromNonRT(ModeRequest{Mode::IDLE, get_node()->now(), false});
        setWheelCommandsRadps(0.0, 0.0);
    }

    bool TerrenceController::isMovingNow() const
    {
        // left/right wheel velocity
        double lv = 0.0, rv = 0.0;
        
        // do the wheels have velocity?
        if (left_vel_state_idx_ >= 0 && right_vel_state_idx_ >= 0)
        {
            lv = state_interfaces_[left_vel_state_idx_].get_value();
            rv = state_interfaces_[right_vel_state_idx_].get_value();
        }
        else // can't use velocity state? use current command instead
        {
            lv = command_interfaces_[left_cmd_idx_].get_value();
            rv = command_interfaces_[right_cmd_idx_].get_value();
        }

        return (std::fabs(lv) > moving_eps_radps_) || (std::fabs(rv) > moving_eps_radps_);
    }

    bool TerrenceController::canTransition(Mode from, Mode to, bool moving_now) const
    {
        if (from == Mode::FAULT)
        {
            return false; // can only exit faults in reset_fault
        }

        if (to == Mode::Fault)
        {
            return true; // technically allowed but just use latchFault() preferably
        }

        // Interlock design:
        // DIG and DUMP require the rover not to be moving
        // DRIVE requires not being digging or dumping
        if ((to == Mode::DIG || to == Mode::DUMP) && moving_now)
            return false;
        
        if ((from == Mode::DIG || from == Mode::DUMP) && to == Mode::DRIVE)
            return false;
        
        return true;
    }

    void TerrenceController::enterMode(Mode new_mode)
    {
        mode_ = new_mode;

        switch(mode_)
        {
            case Mode::IDLE:
                setWheelCommandsRadps(0.0, 0.0);
                // TODO: attachment logic
                break;
            case Mode::DRIVE:
                // TODO: attachment logic
                break;
            case Mode::DIG:
                setWheelCommandsRadps(0.0, 0.0);
                //TODO: attachment logic
                break;
            case Mode::DUMP:
                setWheelCommandsRadps(0.0, 0.0);
                // TODO: attachment logic
                break;
            default:
                setWheelCommandsRadps(0.0, 0.0);
                break;
        }

        RCLCPP_INFO(get_node()->get_logger(), "Mode -> %s", modeToString(mode_).c_str());
    }

    void TerrenceController::setWheelCommandsRadps(double left_radps, double right_radps)
    {
        // clamp to max
        left_radps = clamp(left_radps, -max_wheel_radps_, max_wheel_radps_);
        right_radps = clamp(right_radps, -max_wheel_radps_, max_wheel_radps_);

        command_interfaces_[left_cmd_idx_].set_value(left_radps);
        command_interfaces_[right_cmd_idx_].set_value(right_radps);
    }

    void TerrenceController::computeWheelRadps(double v_mps, double w_radps, double & out_left, double & out_right) const
    {
        // Differential drive kinematics:
        // w_left = (v -  w*L/2)/R
        // w_right = (v + w*L/2)/R

        const double half_L = wheel_separation_m_ * 0.5;
        out_left = (v_mps - w_radps * half_L) / wheel_radius_m_;
        out_right = (v_mps + w_radps * half_L) / wheel_radius_m_;
    }

    // THE UPDATE FUNCTION
    controller_interface::return_type TerrenceController::update(const rclcpp::Time & time, const rclcpp::Duration &)
    {
        // If fault latched, enforce safe outputs
        if (faul_latched_ || mode_ == Mode::FAULT)
        {
            setWheelCommandsRadps(0.0, 0.0);
            // TODO: attachment logic
            return controller_interface::return_type::OK;
        }

        // Mode requests
        const bool moving_now = isMovingNow();
        const auto mode_req = *rt_mode_req_.readFromRT();
        if (mode_req.valid)
        {
            // consume request and make it invalid since it has been consumed
            rt_mode_req_.writeFromNonRT(ModeRequest{mode_req.requested, time, false});

            if (canTransition(mode_, mode_req.requested, moving_now))
            {
                enterMode(mode_req.requested);
            }
            else
            {
            RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                        "Rejected transition %s -> %s (moving_now=%s)",
                        modeToString(mode_).c_str(),
                        modeToString(mode_req.requested).c_str(),
                        moving_now ? "true" : "false");
            }
        }

        // Fulfilled/rejected mode request, apply mode logic
        switch (mode_)
        {
            case Mode::IDLE:
            {
                setWheelCommandsRadps(0.0, 0.0);
                // TODO: attachment logic
                break;
            }

            case Mode::DRIVE:
            {
                const auto cmd = *rt_cmd_vel_.readFromRT();
                const bool stale = (!cmd.valid) || ((time - cmd.stamp).seconds() > cmd_timeout_s_);

                if (stale)
                {
                    // Deadman: stop wheels if cmd_vel is stale
                    setWheelCommandsRadps(0.0, 0.0);
                    break;
                }

                double wl = 0.0, wr = 0.0;
                computeWheelRadpsFromTwist(cmd.linear_x, cmd.angular_z, wl, wr);
                setWheelCommandsRadps(wl, wr);

                // Interlock: do NOT dig while driving
                // TODO: attachment logic

                break;
            }

            case Mode::DIG:
            {
                setWheelCommandsRadps(0.0, 0.0);
                const auto dig = *rt_dig_cmd_.readFromRT();
                (void)dig;
                // TODO: attachment logic
                break;
            }

            case Mode::DUMP:
            {
                setWheelCommandsRadps(0.0, 0.0);
                //TODO: attachment logic
                break;
            }

            case Mode::FAULT:
            default:
            {
                setWheelCommandsRadps(0.0, 0.0);
                break;
            }
        }

        // maybe open loop odom here?
        return controller_interface::return_type::OK;
    }

} // namespace terrence_controller

PLUGINLIB_EXPORT_CLASS(terrence_controller::TerrenceSupervisorController,
                       controller_interface::ControllerInterface)