#include "spacenav_teleop_needle_controller/spacenav_teleop_controller.hpp"

#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"

namespace needle_controllers
{

    SpacenavTeleopController::SpacenavTeleopController()
        : controller_interface::ControllerInterface()
    {
    }

    controller_interface::CallbackReturn SpacenavTeleopController::on_init()
    {
        try
        {
            auto_declare<std::string>("robot_description", "");
            auto_declare<std::string>("robot_base_link", "");
            auto_declare<std::string>("end_effector_link", "");
            auto_declare<std::string>("interface_name", "velocity");
            auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());

            initialized_ = true;
            RCLCPP_INFO(get_node()->get_logger(), "SpacenavTeleopController initialized successfully");
            return controller_interface::CallbackReturn::SUCCESS;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during init stage with message: %s", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }
    }

    controller_interface::CallbackReturn SpacenavTeleopController::on_configure(
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

        // Setup subscription
        tgt_velocity_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_tip", 10,
            std::bind(&SpacenavTeleopController::targetVelocityCallback, this, std::placeholders::_1));

        configured_ = true;
        RCLCPP_INFO(get_node()->get_logger(), "SpacenavTeleopController configured successfully");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn SpacenavTeleopController::on_activate(
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

        // Initialize command
        simulated_joint_cmd_.setZero();
        writeJointControlCmds();

        RCLCPP_INFO(get_node()->get_logger(), "SpacenavTeleopController activated");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn SpacenavTeleopController::on_deactivate(
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

            RCLCPP_INFO(get_node()->get_logger(), "SpacenavTeleopController deactivated");
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

    controller_interface::return_type SpacenavTeleopController::update(
        const rclcpp::Time &, const rclcpp::Duration &)
    {
        writeJointControlCmds();
        return controller_interface::return_type::OK;
    }

    void SpacenavTeleopController::targetVelocityCallback(
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

        // Map twist message to joint commands
        simulated_joint_cmd_[0] = -msg->linear.y; // X axis mapped to Y
        simulated_joint_cmd_[1] = msg->linear.x;  // Y axis mapped to X
        simulated_joint_cmd_[2] = msg->linear.z;  // Z axis as is

        RCLCPP_DEBUG(
            get_node()->get_logger(),
            "Joint velocity commands: [%.3f, %.3f, %.3f]",
            simulated_joint_cmd_[0], simulated_joint_cmd_[1], simulated_joint_cmd_[2]);
    }

    void SpacenavTeleopController::writeJointControlCmds()
    {
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
    SpacenavTeleopController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        for (const auto &joint_name : joint_names_)
        {
            config.names.push_back(joint_name + "/" + cmd_interface_type_);
        }

        return config;
    }

    controller_interface::InterfaceConfiguration
    SpacenavTeleopController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        for (const auto &joint_name : joint_names_)
        {
            config.names.push_back(joint_name + "/position");
        }

        return config;
    }

} // namespace needle_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    needle_controllers::SpacenavTeleopController, controller_interface::ControllerInterface)