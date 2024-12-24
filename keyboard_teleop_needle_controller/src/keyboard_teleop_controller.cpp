#include "keyboard_teleop_needle_controller/keyboard_teleop_controller.hpp"

#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"

namespace needle_controllers
{

    KeyboardTeleopController::KeyboardTeleopController()
        : controller_interface::ControllerInterface()
    {
    }

    controller_interface::CallbackReturn KeyboardTeleopController::on_init()
    {
        try
        {
            // Declare parameters
            auto_declare<std::string>("robot_description", "");
            auto_declare<std::string>("robot_base_link", "");
            auto_declare<std::string>("end_effector_link", "");
            auto_declare<std::string>("interface_name", "velocity");
            auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());

            initialized_ = true;
            RCLCPP_INFO(get_node()->get_logger(), "KeyboardTeleopController initialized successfully");
            return controller_interface::CallbackReturn::SUCCESS;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(
                get_node()->get_logger(),
                "Exception thrown during init stage with message: %s", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }
    }

    controller_interface::CallbackReturn KeyboardTeleopController::on_configure(
        const rclcpp_lifecycle::State &)
    {
        if (!initialized_)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Controller not initialized");
            return controller_interface::CallbackReturn::ERROR;
        }

        // Get parameters
        robot_description_ = get_node()->get_parameter("robot_description").as_string();
        if (robot_description_.empty())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'robot_description' is empty");
            return controller_interface::CallbackReturn::ERROR;
        }

        robot_base_link_ = get_node()->get_parameter("robot_base_link").as_string();
        if (robot_base_link_.empty())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'robot_base_link' is empty");
            return controller_interface::CallbackReturn::ERROR;
        }

        end_effector_link_ = get_node()->get_parameter("end_effector_link").as_string();
        if (end_effector_link_.empty())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'end_effector_link' is empty");
            return controller_interface::CallbackReturn::ERROR;
        }

        joint_names_ = get_node()->get_parameter("joints").as_string_array();
        if (joint_names_.empty())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'joints' is empty");
            return controller_interface::CallbackReturn::ERROR;
        }

        cmd_interface_type_ = get_node()->get_parameter("interface_name").as_string();
        if (cmd_interface_type_.empty())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'interface_name' is empty");
            return controller_interface::CallbackReturn::ERROR;
        }

        // Setup subscription to keyboard-generated Twist messages
        tgt_velocity_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, // Using cmd_vel as the default topic for keyboard teleop
            std::bind(&KeyboardTeleopController::targetVelocityCallback, this, std::placeholders::_1));

        configured_ = true;
        RCLCPP_INFO(get_node()->get_logger(), "KeyboardTeleopController configured successfully");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn KeyboardTeleopController::on_activate(
        const rclcpp_lifecycle::State &)
    {
        if (!configured_)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Controller not configured");
            return controller_interface::CallbackReturn::ERROR;
        }

        // Get command handles
        if (!controller_interface::get_ordered_interfaces(
                command_interfaces_, joint_names_, cmd_interface_type_, joint_cmd_vel_handles_))
        {
            RCLCPP_ERROR(
                get_node()->get_logger(),
                "Expected %zu '%s' command interfaces, got %zu",
                joint_names_.size(), cmd_interface_type_.c_str(),
                joint_cmd_vel_handles_.size());
            return controller_interface::CallbackReturn::ERROR;
        }

        // Get state handles
        if (!controller_interface::get_ordered_interfaces(
                state_interfaces_, joint_names_, hardware_interface::HW_IF_POSITION,
                joint_state_pos_handles_))
        {
            RCLCPP_ERROR(
                get_node()->get_logger(),
                "Expected %zu position state interfaces, got %zu",
                joint_names_.size(),
                joint_state_pos_handles_.size());
            return controller_interface::CallbackReturn::ERROR;
        }

        // Initialize command vector
        simulated_joint_cmd_.setZero();
        writeJointControlCmds();

        RCLCPP_INFO(get_node()->get_logger(), "KeyboardTeleopController activated successfully");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn KeyboardTeleopController::on_deactivate(
        const rclcpp_lifecycle::State &)
    {
        try
        {
            // Reset commands to zero
            simulated_joint_cmd_.setZero();
            writeJointControlCmds();

            // Clear interfaces
            joint_cmd_vel_handles_.clear();
            joint_state_pos_handles_.clear();
            release_interfaces();

            RCLCPP_INFO(get_node()->get_logger(), "KeyboardTeleopController deactivated");
            return controller_interface::CallbackReturn::SUCCESS;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(
                get_node()->get_logger(),
                "Exception thrown during deactivation with message: %s", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }
    }

    controller_interface::return_type KeyboardTeleopController::update(
        const rclcpp::Time &, const rclcpp::Duration &)
    {
        // Write the latest command values to hardware interfaces
        writeJointControlCmds();
        return controller_interface::return_type::OK;
    }

    void KeyboardTeleopController::targetVelocityCallback(
        const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Check for NaN values
        if (std::isnan(msg->linear.x) || std::isnan(msg->linear.y) || std::isnan(msg->linear.z))
        {
            RCLCPP_WARN_THROTTLE(
                get_node()->get_logger(),
                *get_node()->get_clock(),
                1000, // 1 second
                "Received NaN in Twist message, ignoring");
            return;
        }

        // Map keyboard-generated Twist message to joint commands
        // Note: This mapping can be adjusted based on preferred keyboard controls
        simulated_joint_cmd_[0] = msg->linear.x; // Forward/backward
        simulated_joint_cmd_[1] = msg->linear.y; // Left/right
        simulated_joint_cmd_[2] = msg->linear.z; // Up/down

        RCLCPP_DEBUG(
            get_node()->get_logger(),
            "Joint velocity commands: [%.3f, %.3f, %.3f]",
            simulated_joint_cmd_[0], simulated_joint_cmd_[1], simulated_joint_cmd_[2]);
    }

    void KeyboardTeleopController::writeJointControlCmds()
    {
        // Write commanded velocities to each joint
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            if (i < joint_cmd_vel_handles_.size())
            {
                joint_cmd_vel_handles_[i].get().set_value(simulated_joint_cmd_[i]);
            }
            else
            {
                RCLCPP_ERROR(
                    get_node()->get_logger(),
                    "Joint command index %zu out of range", i);
            }
        }
    }

    controller_interface::InterfaceConfiguration
    KeyboardTeleopController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        // Register velocity command interfaces for each joint
        for (const auto &joint_name : joint_names_)
        {
            config.names.push_back(joint_name + "/" + cmd_interface_type_);
        }

        return config;
    }

    controller_interface::InterfaceConfiguration
    KeyboardTeleopController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        // Register position state interfaces for each joint
        for (const auto &joint_name : joint_names_)
        {
            config.names.push_back(joint_name + "/position");
        }

        return config;
    }

} // namespace needle_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    needle_controllers::KeyboardTeleopController, controller_interface::ControllerInterface)