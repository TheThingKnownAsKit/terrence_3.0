#include "control/terrence_controller.hpp"

#include <algorithm>
#include <optional>

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
            auto_declare<std::string>("right_joint_name", "PS_Joint");
            auto_declare<std::string>("loader_joint_name", "LoaderJoint");
            auto_declare<std::string>("hopper_joint_name", "HopperJoint");

            auto_declare<double>("wheel_radius_m", 0.085);
            auto_declare<double>("wheel_separation_m", 0.42);

            auto_declare<double>("max_wheel_radps", 10.0);
            auto_declare<double>("cmd_timeout_s", 0.25);
            auto_declare<double>("moving_eps_radps", 0.05);

            auto_declare<std::string>("odom_topic", "/odom");
            auto_declare<std::string>("odom_frame_id", "odom");
            auto_declare<std::string>("base_frame_id", "base_link");
            auto_declare<bool>("publish_odom_tf", true);
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
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names = {
            left_joint_name_ + "/velocity",
            right_joint_name_ + "/velocity",
            loader_joint_name_ + "/position",
            hopper_joint_name_ + "/position"
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
            right_joint_name_ + "/velocity",
            loader_joint_name_ + "/position",
            hopper_joint_name_ + "/position"
        };

        return config;
    }

    controller_interface::CallbackReturn TerrenceController::on_configure(const rclcpp_lifecycle::State &)
    {
        left_joint_name_ = get_node()->get_parameter("left_joint_name").as_string();
        right_joint_name_ = get_node()->get_parameter("right_joint_name").as_string();
        loader_joint_name_ = get_node()->get_parameter("loader_joint_name").as_string();
        hopper_joint_name_ = get_node()->get_parameter("hopper_joint_name").as_string();

        wheel_radius_m_ = get_node()->get_parameter("wheel_radius_m").as_double();
        wheel_separation_m_ = get_node()->get_parameter("wheel_separation_m").as_double();

        max_wheel_radps_ = get_node()->get_parameter("max_wheel_radps").as_double();
        cmd_timeout_s_ = get_node()->get_parameter("cmd_timeout_s").as_double();
        moving_eps_radps_ = get_node()->get_parameter("moving_eps_radps").as_double();

        odom_topic_ = get_node()->get_parameter("odom_topic").as_string();
        odom_frame_id_ = get_node()->get_parameter("odom_frame_id").as_string();
        base_frame_id_ = get_node()->get_parameter("base_frame_id").as_string();
        publish_odom_tf_ = get_node()->get_parameter("publish_odom_tf").as_bool();

        // Initialize the realtime buffers
        rt_cmd_vel_.writeFromNonRT(CmdVel{0.0, 0.0, get_node()->now(), false});
        rt_dig_cmd_.writeFromNonRT(DigCmd{});
        rt_mode_req_.writeFromNonRT(ModeRequest{Mode::IDLE, get_node()->now(), false});

        // ROS subscriptions
        cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", rclcpp::SystemDefaultsQoS(),
            [this](const geometry_msgs::msg::Twist & msg) { cmdVelCb(msg); });

        dig_cmd_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/dig_cmd", rclcpp::SystemDefaultsQoS(),
            [this](const std_msgs::msg::Float64MultiArray & msg) { digCmdCb(msg); });
        
        // This could be a service but it's easier as a topic rn
        set_mode_sub_ = get_node()->create_subscription<std_msgs::msg::String>(
            "/set_mode", rclcpp::SystemDefaultsQoS(),
            [this](const std_msgs::msg::String & msg) { setModeCb(msg); });

        reset_fault_srv_ = get_node()->create_service<std_srvs::srv::Trigger>(
            "/reset_fault",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
            {
                clearFault();
                resp->success = true;
                resp->message = "Fault cleared; mode set to IDLE.";
            });
        
        // ROS publishers
        auto odom_pub = get_node()->create_publisher<nav_msgs::msg::Odometry>(
            odom_topic_, rclcpp::SystemDefaultsQoS());

        odom_pub_rt_ = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
            odom_pub);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(get_node());

        
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
        loader_cmd_idx_ = hopper_cmd_idx_ = -1;

        left_pos_state_idx_ = left_vel_state_idx_ = -1;
        right_pos_state_idx_ = right_vel_state_idx_ = -1;
        loader_pos_state_idx_ = hopper_pos_state_idx_ = -1;

        x_ = 0.0;
        y_ = 0.0;
        yaw_ = 0.0;

        for (size_t i = 0; i < command_interfaces_.size(); ++i)
        {
            const auto & ci = command_interfaces_[i];
            const auto & jn = ci.get_prefix_name(); // joint name
            const auto & in = ci.get_interface_name(); // interface name

            if (jn == left_joint_name_ && in == "velocity") left_cmd_idx_ = (int)i;
            if (jn == right_joint_name_ && in == "velocity") right_cmd_idx_ = (int)i;
            if (jn == loader_joint_name_ && in == "position") loader_cmd_idx_ = (int)i;
            if (jn == hopper_joint_name_ && in == "position") hopper_cmd_idx_ = (int)i;
        }

        for (size_t i = 0; i < state_interfaces_.size(); ++i)
        {
            const auto & si = state_interfaces_[i];
            const auto & jn = si.get_prefix_name(); // joint name
            const auto & in = si.get_interface_name(); // interface name

            if (jn == left_joint_name_ && in == "position") left_pos_state_idx_ = (int)i;
            if (jn == left_joint_name_ && in == "velocity") left_vel_state_idx_ = (int)i;
            if (jn == right_joint_name_ && in == "position") right_pos_state_idx_ = (int)i;
            if (jn == right_joint_name_ && in == "velocity") right_vel_state_idx_ = (int)i;
            if (jn == loader_joint_name_ && in == "position") loader_pos_state_idx_ = (int)i;
            if (jn == hopper_joint_name_ && in == "position") hopper_pos_state_idx_ = (int)i;
        }

        if (left_cmd_idx_ < 0 || right_cmd_idx_ < 0 || loader_cmd_idx_ < 0 || hopper_cmd_idx_ < 0)
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
        loader_cmd_val_ = 0.0;
        hopper_cmd_val_ = 0.0;
        enterMode(Mode::DRIVE);

        RCLCPP_INFO(get_node()->get_logger(), "Activated TerrenceController.");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn TerrenceController::on_deactivate(const rclcpp_lifecycle::State &)
    {
        // Stop outputs
        setWheelCommandsRadps(0.0, 0.0);
        enterMode(Mode::IDLE);
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

    void TerrenceController::setModeCb(const std_msgs::msg::String & msg)
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
            const auto lv_opt = state_interfaces_[left_vel_state_idx_].get_optional();
            const auto rv_opt = state_interfaces_[right_vel_state_idx_].get_optional();

            // If optional is empty, treat as 0.0
            lv = lv_opt.value_or(0.0);
            rv = rv_opt.value_or(0.0);
        }
        else // can't use velocity state? use current command instead
        {
            // Open-loop estimate
            const auto lv_opt = command_interfaces_[left_cmd_idx_].get_optional();
            const auto rv_opt = command_interfaces_[right_cmd_idx_].get_optional();

            lv = lv_opt.value_or(0.0);
            rv = rv_opt.value_or(0.0);
        }

        return (std::fabs(lv) > moving_eps_radps_) || (std::fabs(rv) > moving_eps_radps_);
    }

    bool TerrenceController::canTransition(Mode from, Mode to, bool moving_now) const
    {
        if (from == Mode::FAULT)
        {
            return false; // can only exit faults in reset_fault
        }

        if (to == Mode::FAULT)
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
        left_radps = std::clamp(left_radps, -max_wheel_radps_, max_wheel_radps_);
        right_radps = std::clamp(right_radps, -max_wheel_radps_, max_wheel_radps_);

        (void)command_interfaces_[left_cmd_idx_].set_value(left_radps);
        (void)command_interfaces_[right_cmd_idx_].set_value(right_radps);
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
    controller_interface::return_type TerrenceController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        // If fault latched, enforce safe outputs
        if (fault_latched_ || mode_ == Mode::FAULT)
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
                loader_cmd_val_ = stowed_loader_cmd_val_;
                hopper_cmd_val_ = stowed_hopper_cmd_val_;
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
                computeWheelRadps(cmd.linear_x, cmd.angular_z, wl, wr);
                setWheelCommandsRadps(wl, wr);

                // Interlock: do NOT dig while driving, set to stowed
                loader_cmd_val_ = stowed_loader_cmd_val_;
                hopper_cmd_val_ = stowed_hopper_cmd_val_;

                break;
            }

            case Mode::DIG:
            {
                setWheelCommandsRadps(0.0, 0.0);
                const auto dig = *rt_dig_cmd_.readFromRT();
                if (dig.valid)
                {
                    // [0] = Loader, [1] = Hopper
                    if (dig.len >= 1) loader_cmd_val_ = dig.data[0];
                    if (dig.len >= 2) hopper_cmd_val_ = dig.data[1];
                }
                break;
            }

            case Mode::DUMP:
            {
                setWheelCommandsRadps(0.0, 0.0);
                const auto dig = *rt_dig_cmd_.readFromRT();
                if (dig.valid)
                {
                    // [0] = Loader, [1] = Hopper
                    if (dig.len >= 1) loader_cmd_val_ = dig.data[0];
                    if (dig.len >= 2) hopper_cmd_val_ = dig.data[1];
                }
                break;
            }

            case Mode::FAULT:
            default:
            {
                setWheelCommandsRadps(0.0, 0.0);
                break;
            }
        }

        // Write commands to interfaces
        (void)command_interfaces_[loader_cmd_idx_].set_value(loader_cmd_val_);
        (void)command_interfaces_[hopper_cmd_idx_].set_value(hopper_cmd_val_);

        // Open loop odom
        double wl = 0.0, wr = 0.0;
        if (left_vel_state_idx_ >= 0 && right_vel_state_idx_ >= 0)
        {
            wl = state_interfaces_[left_vel_state_idx_].get_optional().value_or(0.0);
            wr = state_interfaces_[right_vel_state_idx_].get_optional().value_or(0.0);
        }
        else
        {
            wl = command_interfaces_[left_cmd_idx_].get_optional().value_or(0.0);
            wr = command_interfaces_[right_cmd_idx_].get_optional().value_or(0.0);
        }

        const double dt_raw = period.seconds();
        const double dt = (std::isfinite(dt_raw) && dt_raw > 0.0 && dt_raw < 0.5) ? dt_raw : 0.0;

        // Convert wheel angular velocity (rad/s) to linear (m/s)
        const double vl = wl * wheel_radius_m_;
        const double vr = wr * wheel_radius_m_;

        // Body-frame velocities
        const double v = 0.5 * (vr + vl);
        const double wz = (wheel_separation_m_ > 1e-6) ? ((vr - vl) / wheel_separation_m_) : 0.0;

        // Integrate pose
        yaw_ += wz * dt;

        // Optional: wrap yaw to [-pi, pi] to avoid unbounded growth
        while (yaw_ > M_PI)  yaw_ -= 2.0 * M_PI;
        while (yaw_ < -M_PI) yaw_ += 2.0 * M_PI;

        x_ += v * std::cos(yaw_) * dt;
        y_ += v * std::sin(yaw_) * dt;

        // yaw -> quaternion (2D)
        const double cy = std::cos(yaw_ * 0.5);
        const double sy = std::sin(yaw_ * 0.5);

        // Publish odom
        if (odom_pub_rt_ && odom_pub_rt_->trylock())
        {
            auto & msg = odom_pub_rt_->msg_;
            msg.header.stamp = time;
            msg.header.frame_id = odom_frame_id_;
            msg.child_frame_id = base_frame_id_;

            msg.pose.pose.position.x = x_;
            msg.pose.pose.position.y = y_;
            msg.pose.pose.position.z = 0.0;

            msg.pose.pose.orientation.x = 0.0;
            msg.pose.pose.orientation.y = 0.0;
            msg.pose.pose.orientation.z = sy;
            msg.pose.pose.orientation.w = cy;

            msg.twist.twist.linear.x = v;
            msg.twist.twist.linear.y = 0.0;
            msg.twist.twist.angular.z = wz;

            odom_pub_rt_->unlockAndPublish();
        }

        // Publish TF odom->base
        if (publish_odom_tf_ && tf_broadcaster_)
        {
            geometry_msgs::msg::TransformStamped tf;
            tf.header.stamp = time;
            tf.header.frame_id = odom_frame_id_;
            tf.child_frame_id = base_frame_id_;
            tf.transform.translation.x = x_;
            tf.transform.translation.y = y_;
            tf.transform.translation.z = 0.0;
            tf.transform.rotation.x = 0.0;
            tf.transform.rotation.y = 0.0;
            tf.transform.rotation.z = sy;
            tf.transform.rotation.w = cy;

            tf_broadcaster_->sendTransform(tf);
        }

        return controller_interface::return_type::OK;
    }

} // namespace terrence_controller

PLUGINLIB_EXPORT_CLASS(terrence_controller::TerrenceController,
                       controller_interface::ControllerInterface)