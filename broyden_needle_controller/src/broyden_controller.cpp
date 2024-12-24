#include <string>
#include <mutex>

#include "controller_interface/controller_interface.hpp"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/parameter.hpp"
#include "broyden_needle_controller/broyden_controller.hpp"
#include "rclcpp/logging.hpp"

namespace needle_controllers
{

    controller_interface::CallbackReturn BroydenController::on_init()
    {
        if (!initialized_)
        {
            auto_declare<std::string>("robot_description", "");
            auto_declare<std::string>("robot_base_link", "");
            auto_declare<std::string>("end_effector_link", "");
            auto_declare<std::string>("interface_name", "");
            auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
            initialized_ = true;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn BroydenController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        if (configured_)
        {
            return controller_interface::CallbackReturn::SUCCESS;
        }

        robot_description_ = get_node()->get_parameter("robot_description").as_string();
        if (robot_description_.empty())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "robot_description is empty");
            return controller_interface::CallbackReturn::ERROR;
        }

        robot_base_link_ = get_node()->get_parameter("robot_base_link").as_string();
        if (robot_base_link_.empty())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "robot_base_link is empty");
            return controller_interface::CallbackReturn::ERROR;
        }

        end_effector_link_ = get_node()->get_parameter("end_effector_link").as_string();
        if (end_effector_link_.empty())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "end_effector_link is empty");
            return controller_interface::CallbackReturn::ERROR;
        }

        joint_names_ = get_node()->get_parameter("joints").as_string_array();
        if (joint_names_.empty())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "joints array is empty");
            return controller_interface::CallbackReturn::ERROR;
        }

        cmd_interface_type_ = get_node()->get_parameter("interface_name").as_string();
        if (cmd_interface_type_.empty())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "No command_interfaces specified");
            return controller_interface::CallbackReturn::ERROR;
        }

        // Create subscriptions with corrected string concatenation
        tgt_point_sub_ = get_node()->create_subscription<geometry_msgs::msg::PointStamped>(
            std::string(get_node()->get_name()) + "/cmd_tip", 10,
            std::bind(&BroydenController::targetPointCallback, this, std::placeholders::_1));

        msr_point_sub_ = get_node()->create_subscription<geometry_msgs::msg::PointStamped>(
            std::string(get_node()->get_name()) + "/msr_tip", 10,
            std::bind(&BroydenController::measuredPointCallback, this, std::placeholders::_1));

        configured_ = true;
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn BroydenController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        if (active_)
        {
            return controller_interface::CallbackReturn::SUCCESS;
        }

        // Change from get_command_interfaces() to command_interfaces_
        if (!controller_interface::get_ordered_interfaces(
                command_interfaces_, joint_names_, cmd_interface_type_, joint_cmd_vel_handles_))
        {
            RCLCPP_ERROR(
                get_node()->get_logger(),
                "Expected %zu '%s' command interfaces, got %zu.",
                joint_names_.size(), cmd_interface_type_.c_str(),
                joint_cmd_vel_handles_.size());
            return controller_interface::CallbackReturn::ERROR;
        }

        // Change from get_state_interfaces() to state_interfaces_
        if (!controller_interface::get_ordered_interfaces(
                state_interfaces_, joint_names_, hardware_interface::HW_IF_POSITION, joint_state_pos_handles_))
        {
            RCLCPP_ERROR(
                get_node()->get_logger(),
                "Expected %zu '%s' state interfaces, got %zu.",
                joint_names_.size(), hardware_interface::HW_IF_POSITION,
                joint_state_pos_handles_.size());
            return controller_interface::CallbackReturn::ERROR;
        }

        // Initialize variables
        simulated_joint_cmd_.setZero();
        x_i.setZero();
        y_i.setZero();
        J = Eigen::Matrix3d::Identity();

        writeJointControlCmds();

        active_ = true;
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn BroydenController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        if (active_)
        {
            joint_cmd_vel_handles_.clear();
            joint_state_pos_handles_.clear();
            this->release_interfaces();
        }
        active_ = false;
        return controller_interface::CallbackReturn::SUCCESS;
    }

    void BroydenController::targetPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr target)
    {
        if (!active_)
        {
            return;
        }

        if (std::isnan(target->point.x) || std::isnan(target->point.y) || std::isnan(target->point.z))
        {
            auto &clock = *get_node()->get_clock();
            RCLCPP_WARN_STREAM_THROTTLE(
                get_node()->get_logger(), clock, 3000,
                "NaN detected in target point. Ignoring input.");
            return;
        }

        if (target->header.frame_id != robot_base_link_)
        {
            auto &clock = *get_node()->get_clock();
            RCLCPP_WARN_THROTTLE(
                get_node()->get_logger(), clock, 3000,
                "Got target point in wrong reference frame. Expected: %s but got %s",
                robot_base_link_.c_str(), target->header.frame_id.c_str());
            return;
        }

        // Store the latest target point
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_target_point_ = target;
    }

    void BroydenController::measuredPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr measured)
    {
        if (!active_)
        {
            return;
        }

        if (std::isnan(measured->point.x) || std::isnan(measured->point.y) || std::isnan(measured->point.z))
        {
            auto &clock = *get_node()->get_clock();
            RCLCPP_WARN_STREAM_THROTTLE(
                get_node()->get_logger(), clock, 3000,
                "NaN detected in measured point. Ignoring input.");
            return;
        }

        if (measured->header.frame_id != robot_base_link_)
        {
            auto &clock = *get_node()->get_clock();
            RCLCPP_WARN_THROTTLE(
                get_node()->get_logger(), clock, 3000,
                "Got measured point in wrong reference frame. Expected: %s but got %s",
                robot_base_link_.c_str(), measured->header.frame_id.c_str());
            return;
        }

        // Store the latest measured point
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_measured_point_ = measured;
    }

    controller_interface::return_type BroydenController::update(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        if (active_)
        {
            Eigen::Vector3d x_j;
            Eigen::Vector3d y_j;

            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                if (!latest_target_point_ || !latest_measured_point_)
                {
                    RCLCPP_WARN_THROTTLE(
                        get_node()->get_logger(), *get_node()->get_clock(), 3000,
                        "Target or measured point not received yet.");
                    return controller_interface::return_type::OK;
                }

                // Assign x_j and y_j from the latest points
                x_j << latest_target_point_->point.x, latest_target_point_->point.y, latest_target_point_->point.z;
                y_j << latest_measured_point_->point.x, latest_measured_point_->point.y, latest_measured_point_->point.z;

                // Reset the latest points after reading
                latest_target_point_.reset();
                latest_measured_point_.reset();
            }

            broydenUpdate(x_j, y_j);
        }

        writeJointControlCmds();
        return controller_interface::return_type::OK;
    }

    void BroydenController::writeJointControlCmds()
    {
        for (std::size_t i = 0; i < joint_names_.size(); ++i)
        {
            joint_cmd_vel_handles_[i].get().set_value(simulated_joint_cmd_(i));
        }
    }

    void BroydenController::broydenUpdate(const Eigen::Vector3d &x_j, const Eigen::Vector3d &y_j)
    {
        // y_t = f(x_t)
        // dy = y_t - y_t-1
        Eigen::Vector3d dx = x_j - x_i;
        Eigen::Vector3d dy = y_j - y_i;

        double dx_dot = dx.dot(dx);
        if (dx_dot > 1e-6) // Avoid division by zero or very small numbers
        {
            J = J + ((dy - J * dx) / dx_dot) * dx.transpose();
        }
        else
        {
            RCLCPP_WARN(get_node()->get_logger(), "dx norm too small, skipping Broyden update.");
            return;
        }

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
        double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);

        if (cond < 1e6)
        {
            Eigen::Vector3d rhs(0, 0.5, 0);
            simulated_joint_cmd_ = svd.solve(rhs);
        }
        else
        {
            RCLCPP_WARN(
                get_node()->get_logger(),
                "Jacobian condition number too high: %f. Skipping command update.", cond);
        }

        // Update previous state
        x_i = x_j;
        y_i = y_j;
    }

    controller_interface::InterfaceConfiguration BroydenController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names.reserve(joint_names_.size());
        for (const auto &joint : joint_names_)
        {
            config.names.emplace_back(joint + "/" + cmd_interface_type_);
        }
        return config;
    }

    controller_interface::InterfaceConfiguration BroydenController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names.reserve(joint_names_.size());
        for (const auto &joint : joint_names_)
        {
            config.names.emplace_back(joint + "/position");
        }
        return config;
    }

} // namespace needle_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(needle_controllers::BroydenController, controller_interface::ControllerInterface)
