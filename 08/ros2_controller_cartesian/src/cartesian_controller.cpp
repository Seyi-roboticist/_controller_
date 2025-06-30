/**
 * @author Seyi R. Afolayan
 * @name Cartesian Controller for SE3 Sensor System (Implementation)
 *
 * @brief Implementation of the position-only Cartesian controller that uses SE3 sensor data
 *
 * @date 14/04/2025
 */

#include "ros2_controller_cartesian/cartesian_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <iomanip>
#include <sstream>
#include <Eigen/SVD>

#include <controller_interface/helpers.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/parameter.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace ros2_controller_cartesian
{
    /////////////////////////////////////////////////CONSTRUCTOR AND DESTRUCTOR////////////////////////////////////////////////////////
    CartesianController::CartesianController() : printFreq(1000) // Prints debug info very 1 second
    {
        // I initialize the logger and clock similar to what I did in assignmnet 6
        this->logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("cartesian_controller")); // cartesian controller logger
        this->clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);

        RCLCPP_INFO(this->get_logger(), "CartesianController created");
    }

    CartesianController::~CartesianController()
    {
        RCLCPP_INFO(this->get_logger(), "CartesianController Destroyed");
    }

    ///////////////////////////////////////////////////////////ON INITIALIZATION///////////////////////////////////////////////////////
    controller_interface::CallbackReturn CartesianController::on_init()
    {
        RCLCPP_INFO(this->get_logger(), "Initializing the CartesianController...");

        try
        {
            // Declare all controller parameters
            auto_declare<std::string>("robot_description", "");
            auto_declare<std::string>("robot_base_link", "base_link"); // this can be overriden
            auto_declare<std::string>("end_effector_link", "tool0");   // this can be overriden
            auto_declare<std::string>("reference_frame", "base_link"); // also overriden if we get like map or base or base_footprint
            auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
            auto_declare<double>("velocity_scaling_factor", 2.0);
            auto_declare<double>("error_threshold", 0.001);
            auto_declare<double>("damping_factor", 0.001);
            auto_declare<std::vector<double>>("position_gain", {1.0, 1.0, 1.0});

            // Transforms as XYZ RPY for parameters --> [x y z r p y]'
            auto_declare<std::vector<double>>("sensor_to_base_transform", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
            auto_declare<std::vector<double>>("tool_to_end_effector_transform", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception thrown during init: %s", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    ////////////////////////////////////////////////////////ON CONFIGURATION//////////////////////////////////////////////////////////////
    controller_interface::CallbackReturn CartesianController::on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "Configuring CartesianController");

        try
        {
            // Get the parameters
            /* robot description param */
            robot_description_ = this->get_node()->get_parameter("robot_description").as_string();
            if (robot_description_.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "Parameter 'robot_description' is empty");
                return controller_interface::CallbackReturn::ERROR;
            }

            /* robot's baselink param */
            robot_base_link_ = this->get_node()->get_parameter("robot_base_link").as_string();
            /* end effector param */
            end_effector_link_ = this->get_node()->get_parameter("end_effector_link").as_string();
            /* Reference param */
            reference_frame_ = this->get_node()->get_parameter("reference_frame").as_string();
            /* joint names */
            joint_names_ = this->get_node()->get_parameter("joints").as_string_array();
            /* velocity scaling factor */
            velocity_scaling_factor_ = this->get_node()->get_parameter("velocity_scaling_factor").as_double();
            /* error threshold */
            error_threshold_ = this->get_node()->get_parameter("error_threshold").as_double();
            /* position gain */
            position_gain_ = this->get_node()->get_parameter("position_gain").as_double_array();
            ki_ = this->get_node()->get_parameter("integral_gain").as_double_array();
            kd_ = this->get_node()->get_parameter("derivative_gain").as_double_array();

            damping_factor_ = this->get_node()->get_parameter("damping_factor").as_double();

            // Initialize publishers for Gazebo integration
            velocity_publishers_.resize(joint_names_.size());
            for (std::size_t i = 0; i < joint_names_.size(); ++i)
            {
                std::string topic_name = "/ur5e/" + joint_names_[i] + "/velocity_controller/command";
                velocity_publishers_[i] = get_node()->create_publisher<std_msgs::msg::Float64>(topic_name, 10);
                RCLCPP_INFO(this->get_logger(), "Created publisher for topic: %s", topic_name.c_str());
            }

            // Validate Parameters
            if (joint_names_.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "No joints specified!");
                return controller_interface::CallbackReturn::ERROR;
            }

            if (position_gain_.size() != 3)
            {
                RCLCPP_ERROR(this->get_logger(), "Position gain must be a 3-element array");
                return controller_interface::CallbackReturn::ERROR;
            }

            // Let's print it out for now
            RCLCPP_INFO(this->get_logger(), "Controller Configuration:");
            RCLCPP_INFO(this->get_logger(), "Base Link: %s", robot_base_link_.c_str());
            RCLCPP_INFO(this->get_logger(), "End Effector Link: %s", end_effector_link_.c_str());
            RCLCPP_INFO(this->get_logger(), "Velocity Scaling: %f", velocity_scaling_factor_);
            RCLCPP_INFO(this->get_logger(), "Error Threshold: %f", error_threshold_);

            // Get transformation parameters
            auto sensor_to_base_params = this->get_node()->get_parameter("sensor_to_base_transform").as_double_array();
            auto tool_to_end_effector_params = this->get_node()->get_parameter("tool_to_end_effector_transform").as_double_array();

            if (sensor_to_base_params.size() != 6 || tool_to_end_effector_params.size() != 6)
            {
                RCLCPP_ERROR(this->get_logger(), "Transform parameters must be 6-element arrays (x,y,z,r,p,y)");
                return controller_interface::CallbackReturn::ERROR;
            }

            // Create KDL frames from transform parameters (x, y, z, r, p, y)
            sensor_to_base_transform_ = KDL::Frame(
                KDL::Rotation::RPY(sensor_to_base_params[3], sensor_to_base_params[4], sensor_to_base_params[5]),
                KDL::Vector(sensor_to_base_params[0], sensor_to_base_params[1], sensor_to_base_params[2]));

            tool_to_end_effector_transform_ = KDL::Frame(
                KDL::Rotation::RPY(tool_to_end_effector_params[3], tool_to_end_effector_params[4], tool_to_end_effector_params[5]),
                KDL::Vector(tool_to_end_effector_params[0], tool_to_end_effector_params[1], tool_to_end_effector_params[2]));

            RCLCPP_INFO(this->get_logger(), "Sensor to Base Transform: [%f, %f, %f, %f, %f, %f]",
                        sensor_to_base_params[0], sensor_to_base_params[1], sensor_to_base_params[2],
                        sensor_to_base_params[3], sensor_to_base_params[4], sensor_to_base_params[5]);
            RCLCPP_INFO(this->get_logger(), "Tool to End Effector Transform: [%f, %f, %f, %f, %f, %f]",
                        tool_to_end_effector_params[0], tool_to_end_effector_params[1], tool_to_end_effector_params[2],
                        tool_to_end_effector_params[3], tool_to_end_effector_params[4], tool_to_end_effector_params[5]);

            // Initialize TF buffer and listener
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_node()->get_clock());
            tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

            // Check Initialize KDL --> call KDL ... if boolean is 0, I will print error
            if (!initKDL())
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to initialize KDL!");
                return controller_interface::CallbackReturn::ERROR;
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception thrown during configure: %s", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }
        RCLCPP_INFO(this->get_logger(), "CartesianController successfully configured!!!");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    /////////////////////////////////////////////////////INITIALIZE KDL///////////////////////////////////////////////////////
    bool CartesianController::initKDL()
    {
        // Parse the robot description into a KDL tree like rspweek07 but I will be doing mine less explicitly (guard clause pattern)
        if (!kdl_parser::treeFromString(robot_description_, robot_tree_))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse the KDL tree from robot description");
            return false;
        }

        // Extract the robot chain --> Base to End-effector
        if (!robot_tree_.getChain(robot_base_link_, end_effector_link_, robot_chain_))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to get KDL chain from tree");
            return false;
        }

        // If the above hypotheticals are skipped, then I should have a full kinematic chain to work with
        RCLCPP_INFO(this->get_logger(), "KDL chain created with %d segments from '%s' to '%s'",
                    robot_chain_.getNrOfSegments(), robot_base_link_.c_str(), end_effector_link_.c_str());

        // We move to create the legendary solvers
        fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(robot_chain_);
        jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(robot_chain_);

        return true;
    }

    /////////////////////////////////////////////////////////ON ACTIVATION//////////////////////////////////////////////////////////////
    controller_interface::CallbackReturn CartesianController::on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "Activating CartesianController...");

        try
        {
            /* We are retrieving hardware interfaces that my controller needs to interact with the robot's joints */
            /* Establishing the connection between my controller and the actual robot hardware */
            /* The position_state_interface lets me read the  read the current joint position */
            /* The velocity_command_interfaces lets me send velocity commands to the joints */

            // Get joint position state interface using guarde clause pattern again
            if (!controller_interface::get_ordered_interfaces(state_interfaces_, joint_names_,
                                                              hardware_interface::HW_IF_POSITION,
                                                              position_state_interfaces_))
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to get position state interfaces!");
                return controller_interface::CallbackReturn::ERROR;
            }

            // Get joint velocity command interface
            if (!controller_interface::get_ordered_interfaces(command_interfaces_, joint_names_,
                                                              hardware_interface::HW_IF_VELOCITY,
                                                              velocity_command_interfaces_))
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to get velocity command interfaces");
                return controller_interface::CallbackReturn::ERROR;
            }

            RCLCPP_INFO(this->get_logger(), "Found all required interfaces:");
            RCLCPP_INFO(this->get_logger(), "  %zu position state interfaces", position_state_interfaces_.size());
            RCLCPP_INFO(this->get_logger(), "  %zu velocity command interfaces", velocity_command_interfaces_.size());

            // Initialize velocity commands to zero
            for (auto &interface : velocity_command_interfaces_)
                interface.get().set_value(0.0);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception thrown during activation: %s", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }
        RCLCPP_INFO(this->get_logger(), "CartesianController successfully activated");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    ///////////////////////////////////////////////////////////ON DEACTIVATION//////////////////////////////////////////////////////
    controller_interface::CallbackReturn CartesianController::on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "Deactivating CartesianController");

        // Set velocity commands to zero before deactivating
        for (auto &interface : velocity_command_interfaces_)
        {
            interface.get().set_value(0.0);
        }

        // Clear interface references
        velocity_command_interfaces_.clear();
        position_state_interfaces_.clear();

        return controller_interface::CallbackReturn::SUCCESS;
    }

    /////////////////////////////////////////////////////ON COMMAND INTERFACE CONFIGURATION//////////////////////////////////////////////
    controller_interface::InterfaceConfiguration CartesianController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;                          // We instantiate a new object that will be returned to the Ctrl Mgr
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL; // specifies each controller individually

        // Build the list of required command interfaces by iterating through each joint name
        for (const auto &joint_name : joint_names_)
            config.names.emplace_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);

        RCLCPP_INFO(this->get_logger(), "Returning command interface configuration with %zu interfaces", config.names.size());
        for (const auto &name : config.names)
        {
            RCLCPP_DEBUG(this->get_logger(), "  Command interface: %s", name.c_str());
        }

        return config;
    }

    /////////////////////////////////////////////////////ON STATE INTERFACE CONFIGURATION//////////////////////////////////////////////
    controller_interface::InterfaceConfiguration CartesianController::state_interface_configuration() const
    {
        /* This method is pretty much the controller's shopping list --> It pretty much tells ROS2 exactly what information my controller needs to read */
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        /* We need to know where the joints currently are */
        // Joint position interfaces: These interfaces gives my controller acccess to read the current position of each joint
        for (const auto &joint_name : joint_names_)
            config.names.emplace_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);

        RCLCPP_INFO(this->get_logger(), "Returning state interface configuration with %zu interfaces", config.names.size());
        for (const auto &name : config.names)
        {
            RCLCPP_DEBUG(this->get_logger(), "  State interface: %s", name.c_str());
        }

        return config;
    }

    ////////////////////////////////////////////////GET CURRENT JOINT POSITIONS//////////////////////////////////////////////////////
    /**
     * @brief This method retrieves the current joint positions from the hardware interface and
     * returns them in a KDL joint array format.
     */
    KDL::JntArray CartesianController::getCurrentJointPositions()
    {
        KDL::JntArray positions(position_state_interfaces_.size());
        for (std::size_t i{0}; i < position_state_interfaces_.size(); ++i)
            positions(i) = position_state_interfaces_[i].get().get_value();

        return positions;
    }

    ///////////////////////////////////////////////////GET SENSOR ROBOT FRAME/////////////////////////////////////////////////////////
    /**
     * @brief Reads the current position and orientation of the robot's sensor from TF and converts it into a KDL Frame.
     */
    KDL::Frame CartesianController::getSensorRobotFrame()
    {
        try
        {
            geometry_msgs::msg::TransformStamped transform =
                tf_buffer_->lookupTransform(reference_frame_, "robot_sensor_frame", tf2::TimePointZero);

            // Convert to KDL frame
            return tf2::transformToKDL(transform);
        }
        catch (const tf2::TransformException &ex)
        {
            // Return identity transform as fallback
            return KDL::Frame::Identity();
        }
    }

    ///////////////////////////////////////////////////GET SENSOR TARGET FRAME/////////////////////////////////////////////////////////
    /**
     * @brief Reads the current position and orientation of the target's sensor from TF and converts it into a KDL Frame.
     */
    KDL::Frame CartesianController::getSensorTargetFrame()
    {
        try
        {
            geometry_msgs::msg::TransformStamped transform =
                tf_buffer_->lookupTransform(reference_frame_, "target_sensor_frame", tf2::TimePointZero);

            // Convert to KDL frame
            return tf2::transformToKDL(transform);
        }
        catch (const tf2::TransformException &ex)
        {
            return KDL::Frame::Identity();
        }
    }

    ///////////////////////////////////////////////////TWIST -> CARTESIAN ERROR////////////////////////////////////////////////////////
    /**
     * @brief For the error twist, I am calculating only position and ignoring orientation for now. Verify with Dr. Leonard
     * @details The method computes the difference between the current and target positions and then converts it into a velocity command.
     * @name Simple proportional controller for position
     * @param current, target
     */
    KDL::Twist CartesianController::calculateCartesianError(const KDL::Frame &current, const KDL::Frame &target, const rclcpp::Duration &period)
    {
        KDL::Twist error;

        // P term: Calculate the position error
        KDL::Vector position_error = target.p - current.p;

        // D term: Calculate derivative of error (change in error)
        KDL::Vector error_derivative = KDL::Vector::Zero();
        double dt = period.seconds();

        if (dt > 0.0)
        {
            error_derivative = (position_error - last_error) / dt;
        }
        last_error = position_error; // Store current error for next cycle

        // I term: Integrate error over time
        error_integral = error_integral + position_error * dt;

        // Anti-windup: Limit integral term to prevent excessive buildup --- @cite Idea from JHU UAV Systems and Control 
        double integral_limit = 1.0; // Adjust as needed
        for (int i = 0; i < 3; i++)
        {
            if (std::abs(error_integral(i)) > integral_limit)
            {
                error_integral(i) = integral_limit * (error_integral(i) > 0 ? 1.0 : -1.0);
            }
        }

        // Apply PID formula
        error.vel = KDL::Vector::Zero();
        for (int i = 0; i < 3; i++)
        {
            int idx = std::min(i, static_cast<int>(position_gain_.size() - 1));
            int ki_idx = std::min(i, static_cast<int>(ki_.size() - 1));
            int kd_idx = std::min(i, static_cast<int>(kd_.size() - 1));

            // PID control law: u = Kp*e + Ki∫e dt + Kd*de/dt
            double control = position_gain_[idx] * position_error(i) +
                             ki_[ki_idx] * error_integral(i) +
                             kd_[kd_idx] * error_derivative(i);

            switch (i)
            {
            case 0:
                error.vel.x(control);
                break;
            case 1:
                error.vel.y(control);
                break;
            case 2:
                error.vel.z(control);
                break;
            }
        }

        // Set rotation error to zero - we're ignoring orientation
        error.rot = KDL::Vector::Zero();

        return error;
    }
    ///////////////////////////////////////////////////CALCULATE JOINT VELOCITIES/////////////////////////////////////////////////////
    /**
     * @brief Converts Cartesian error to joint velocities using the legendary manipulator jacobian
     * @details Calculates the Jacobian matrix at the current configuration, computes its pseudo-inverse,
     *          and multiplies by the Cartesian error to get joint velocities.
     *          Uses SVD (Singular Value Decomposition) for a numerically stable inverse --> Just like ABSR assignment1
     * @param cartesian_error The Cartesian velocity needed (Twist)
     * @param joint_velocities Output parameter that will be filled with calculated joint velocities
     * @return True if calculation succeeded, false if it failed (e.g., Jacobian calculation error)
     */
    bool CartesianController::calculateJointVelocities(const KDL::Twist &cartesian_error, KDL::JntArray &joint_velocities)
    {
        // Get current joint positions
        KDL::JntArray joint_positions = getCurrentJointPositions();

        // Resize joint velocities
        joint_velocities.resize(joint_positions.rows());

        // Create Jacobian
        KDL::Jacobian jacobian(joint_positions.rows());

        // Calculate Jacobian
        int ret = jac_solver_->JntToJac(joint_positions, jacobian);
        if (ret != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to compute Jacobian: %d", ret);
            return false;
        }

        // Extract velocity components from cartesian error
        Eigen::Vector3d vel;
        vel(0) = cartesian_error.vel.x();
        vel(1) = cartesian_error.vel.y();
        vel(2) = cartesian_error.vel.z();

        // Extract position part of Jacobian (first 3 rows)
        Eigen::MatrixXd jac_position = jacobian.data.block(0, 0, 3, jacobian.columns());

        // Use SVD for numerical stability
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(jac_position, Eigen::ComputeThinU | Eigen::ComputeThinV);

        // Create damped pseudo-inverse with fixed damping factor
        Eigen::MatrixXd s_inv = Eigen::MatrixXd::Zero(svd.matrixV().cols(), svd.matrixU().cols());
        Eigen::VectorXd s = svd.singularValues();

        // Fill in the inverted singular values with damping
        for (Eigen::Index i = 0; i < s.size(); ++i)
        {
            s_inv(i, i) = s(i) / (s(i) * s(i) + damping_factor_ * damping_factor_);
        }

        // Calculate damped pseudo-inverse
        Eigen::MatrixXd J_pinv = svd.matrixV() * s_inv * svd.matrixU().transpose();

        // Calculate joint velocities
        Eigen::VectorXd qdot = J_pinv * vel;

        // Apply velocity scaling
        qdot *= velocity_scaling_factor_;

        // Copy to output
        for (Eigen::Index i = 0; i < joint_velocities.rows(); ++i)
        {
            joint_velocities(i) = (i < qdot.size()) ? qdot(i) : 0.0;
        }

        return true;
    }
    ////////////////////////////////////////////////////WRITE JOINT VELOCITIES////////////////////////////////////////////////////////
    void CartesianController::writeJointVelocities(const KDL::JntArray &joint_velocities)
    {
        // For debugging, print the joint velocities periodically
        if (printFreq.shouldPrint())
        {
            // Create a string stream for logging
            std::stringstream ss;
            ss << "Joint velocities: ";

            for (std::size_t i{0}; i < velocity_command_interfaces_.size(); ++i)
            {
                double vel = (i < joint_velocities.rows()) ? joint_velocities(i) : 0.0;
                ss << vel << " ";
            }

            RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
        }

        // Write the joint velocities to the command interfaces
        for (std::size_t i{0}; i < std::min(static_cast<std::size_t>(joint_velocities.rows()), velocity_command_interfaces_.size()); ++i)
        {
            this->velocity_command_interfaces_[i].get().set_value(joint_velocities(i));

            // Publish to topics for Gazebo integration
            if (i < velocity_publishers_.size() && velocity_publishers_[i])
            {
                auto msg = std::make_unique<std_msgs::msg::Float64>();
                msg->data = joint_velocities(i);
                velocity_publishers_[i]->publish(std::move(msg));
            }
        }
    }

    ///////////////////////////////////////////////////UPDATE (MAIN CONTROL LOOP)/////////////////////////////////////////////////////
    /**
     * @brief The main control loop that runs each control cycle
     * @details Gets the current robot and target poses from sensors, calculates the position error,
     *          converts this error to joint velocities, and sends commands to the robot.
     *          This implements a Cartesian position controller using inverse kinematics.
     * @param time Current ROS time
     * @param period Time since last update (not used in this implementation)
     * @return SUCCESS if the controller update succeeded, ERROR if any exception occurred
     */
    controller_interface::return_type CartesianController::update(const rclcpp::Time &, const rclcpp::Duration &period)
    {
        static bool target_reached = false;

        try
        {
            // Get current sensor frames (aka the poses)
            KDL::Frame robot_sensor_frame = this->getSensorRobotFrame();
            KDL::Frame target_sensor_frame = this->getSensorTargetFrame();

            /* We need to perform coordinate transformation */
            // Transform robot sensor frame to base frame using sensor_to_base_transform_
            KDL::Frame robot_tool_frame_in_base = this->sensor_to_base_transform_ * robot_sensor_frame;

            // Transform target sensor frame to base frame using sensor_to_base_transform_
            KDL::Frame target_frame_in_base = this->sensor_to_base_transform_ * target_sensor_frame;

            // Calculate position error directly using vector subtraction
            KDL::Vector position_error = target_frame_in_base.p - robot_tool_frame_in_base.p;
            double error_norm = position_error.Norm();

            // Get current joint positions for forward kinematics
            KDL::JntArray joint_positions = getCurrentJointPositions();
            KDL::Frame robot_fk_pose;

            // Calculate forward kinematics to get actual robot position
            fk_solver_->JntToCart(joint_positions, robot_fk_pose);

            // Calculate error based on robot position and target
            KDL::Vector fk_error = target_frame_in_base.p - robot_fk_pose.p;
            double fk_error_norm = fk_error.Norm();

            // Every 100 cycles, print detailed debug info
            static int debug_counter = 0;
            if (++debug_counter % 100 == 0)
            {
                RCLCPP_INFO(this->get_logger(), "DEBUG: Sensor Error: %.6f, FK Error: %.6f, Threshold: %.6f",
                            error_norm, fk_error_norm, error_threshold_);
                RCLCPP_INFO(this->get_logger(), "DEBUG: Position - Target: [%.6f, %.6f, %.6f], Sensor Tool: [%.6f, %.6f, %.6f], Robot Tool: [%.6f, %.6f, %.6f]",
                            target_frame_in_base.p.x(), target_frame_in_base.p.y(), target_frame_in_base.p.z(),
                            robot_tool_frame_in_base.p.x(), robot_tool_frame_in_base.p.y(), robot_tool_frame_in_base.p.z(),
                            robot_fk_pose.p.x(), robot_fk_pose.p.y(), robot_fk_pose.p.z());
            }

            // Check if target is reached using FK error
            if (fk_error_norm <= error_threshold_)
            {
                // Target reached - explicitly set all velocities to zero
                for (auto &interface : velocity_command_interfaces_)
                {
                    interface.get().set_value(0.0);
                }

                // Reset integral term when target is reached
                error_integral = KDL::Vector::Zero();

                // Log once when target is reached
                if (!target_reached)
                {
                    RCLCPP_INFO(this->get_logger(), "TARGET REACHED! FK Error: %f m", fk_error_norm);
                    target_reached = true;
                }

                return controller_interface::return_type::OK;
            }

            // Reset log flag if we move away from target
            if (fk_error_norm > error_threshold_ * 1.5 && target_reached)
            {
                RCLCPP_INFO(this->get_logger(), "Moving away from target. FK Error: %f m", fk_error_norm);
                target_reached = false;
            }

            // Calculate the cartesian error using sensor data with PID control
            KDL::Twist cartesian_error = this->calculateCartesianError(robot_tool_frame_in_base, target_frame_in_base, period);

            // Calculate the joint velocities from the cartesian error
            KDL::JntArray joint_velocities(joint_names_.size());
            if (!this->calculateJointVelocities(cartesian_error, joint_velocities))
            {
                // Failed to calculate joint velocities, stop the robot
                for (auto &interface : velocity_command_interfaces_)
                {
                    interface.get().set_value(0.0);
                }

                // Reset integral term to prevent buildup
                error_integral = KDL::Vector::Zero();

                return controller_interface::return_type::OK;
            }

            // Write joint velocities to the command interfaces
            writeJointVelocities(joint_velocities);

            // Print debug information periodically
            if (printFreq.shouldPrint())
            {
                std::stringstream ss;
                ss << std::fixed << std::setprecision(5);
                ss << "Cartesian Controller Status:" << std::endl
                   << "  Sensor Error: " << error_norm << " m, FK Error: " << fk_error_norm << " m" << std::endl
                   << "  Target: [" << target_frame_in_base.p.x() << ", "
                   << target_frame_in_base.p.y() << ", "
                   << target_frame_in_base.p.z() << "]" << std::endl
                   << "  Robot Tool (FK): [" << robot_fk_pose.p.x() << ", "
                   << robot_fk_pose.p.y() << ", "
                   << robot_fk_pose.p.z() << "]" << std::endl
                   << "  Integral Term: [" << error_integral.x() << ", "
                   << error_integral.y() << ", "
                   << error_integral.z() << "]";

                RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
            }
            return controller_interface::return_type::OK;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception thrown during controller update: %s", e.what());

            // In case of error, stop the robot
            for (auto &interface : velocity_command_interfaces_)
            {
                interface.get().set_value(0.0);
            }

            // Reset integral term to prevent buildup on errors
            error_integral = KDL::Vector::Zero();

            return controller_interface::return_type::ERROR;
        }
    }
}

PLUGINLIB_EXPORT_CLASS(ros2_controller_cartesian::CartesianController, controller_interface::ControllerInterface)