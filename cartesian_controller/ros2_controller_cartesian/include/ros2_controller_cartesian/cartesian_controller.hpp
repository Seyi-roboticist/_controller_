/**
 * @author Seyi R. Afolayan
 * @name Cartesian Controller for SE3 Sensor System  (Header)
 *
 * @brief A position-only Cartesian controller that uses external SE3 sensor data to drive a robot to a target.
 *
 * @date 14/04/2025
 */

#if !defined(CARTESIAN_CONTROLLER_HPP)
#define CARTESIAN_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_kdl/tf2_kdl.hpp>

#include <std_msgs/msg/float64.hpp>  // For Float64 message typ

namespace ros2_controller_cartesian
{
    /**
     * @brief Utility class to throttle debug prints
     */
    class PrintFrequency
    {
    public:
        /**
         * @brief Constructor with configurable interval
         * @param intervalMs Interval in milliseconds between prints
         */
        PrintFrequency(int intervalMs = 1000) : interval(std::chrono::milliseconds(intervalMs))
        {
            lastPrintTime = std::chrono::steady_clock::now();
        }

        /**
         * @brief Check if enough time has passed to print again
         * @return true if should print, false otherwise
         */
        bool shouldPrint()
        {
            auto now = std::chrono::steady_clock::now();
            if (now - lastPrintTime >= interval)
            {
                lastPrintTime = now;
                return true;
            }
            return false;
        }

    private:
        std::chrono::steady_clock::time_point lastPrintTime;
        std::chrono::milliseconds interval;
    };

    /**
     * @brief Cartesian controller that uses external sensor data to control a robot
     */
    class CartesianController : public controller_interface::ControllerInterface
    {
    public:
        // Constructor and destructor
        CartesianController();
        ~CartesianController();

        // Lifecycle Methods
        controller_interface::CallbackReturn on_init() override;
        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
        controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

        // The main control method
        controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &) override;

        // Interface configuration methods
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        /// @brief Unlike the node class, there is no get_logger method. So we need to manually create it.
        /// @return Logger of the SensorInterface
        rclcpp::Logger get_logger() const { return *logger_; }

        /// @brief Get the clock of the SensorInterface
        /// @return Logger of the Sensor Interface
        rclcpp::Clock::SharedPtr get_clock() const { return clock_; }

    protected:
        /**
         * @brief // To help initialize the KDL components --> Parses the URDF to create the KDL tree and Chain. Setup fk and Jac solvers.
         * @returns Boolean indicatin success or failure
         */
        bool initKDL();

        /**
         * @brief Retrieves current joint positions from the hardware interface
         * @returns A KDL joint array containing the current joint positions
         */
        KDL::JntArray getCurrentJointPositions();

        /**
         * @brief Get the target pose from the external sensor
         * @returns Frame representing the target pose
         */
        KDL::Frame getSensorTargetFrame();

        /**
         * @brief Get the robot's current tool pose from the external sensor
         * @returns Frame representing the current tool pose
         */
        KDL::Frame getSensorRobotFrame();

        /**
         * @brief Calculates the error between the two poses
         * @param current, target
         * @details Computes the difference between position and orientations, applies gains (might take off orientation later)
         * @returns twist representing the position and error
         */
        KDL::Twist calculateCartesianError(const KDL::Frame &current, const KDL::Frame &target, const rclcpp::Duration &period);

        /**
         * @brief Converts Cartesian error to joint velocities
         * @param cartesian, joint_velocities
         * @details Calculates the Jacobian matrix, computes its psuedo-inverse, and multiplies by the error
         * @returns Boolean indicating success or failure
         */
        bool calculateJointVelocities(const KDL::Twist &cartesian_error, KDL::JntArray &joint_velocities);

        /**
         * @brief Sends the calculated joint velocities to the robot hardware like Leonard talked about
         * @details Write values to velocity_command_interfaces_
         * @param joint_velocities
         */
        void writeJointVelocities(const KDL::JntArray &joint_velocities);

        // Robot model parameters
        std::string robot_description_; // URDF
        std::string robot_base_link_;   // Baselink
        std::string end_effector_link_; // tool0 area (like in the UR5e)
        std::string reference_frame_;   // Like world
        KDL::Tree robot_tree_;          // KDL representation of the entire robot structure
        KDL::Chain robot_chain_;        // Specific kinematic chain from baselink to end-effector

        // Solvers
        std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_; // Recursive algorithm to compute FK
        std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;       // Computes the Jacobian matric for the kinematic chain

        // Controller Parameters
        double velocity_scaling_factor_;    // Scales the joint velocities (similar to the demo for ASBR --> might be between 0.0 and 1.0)
        double error_threshold_;            // The precision error we need to get to within the target location to consider it "reached"
        std::vector<double> position_gain_; // Kp gains on [x y z] --> How aggressively it responds to position errors

        // Joint/Sensor interface related
        std::vector<std::string> joint_names_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> velocity_command_interfaces_; // borrowed from HW
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> position_state_interfaces_;     // borrowed from HW

        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

        // Sensor state interfaces - these get the SE3 sensor data
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> robot_sensor_state_interfaces_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> target_sensor_state_interfaces_;

        // Transformations
        KDL::Frame sensor_to_base_transform_;
        KDL::Frame tool_to_end_effector_transform_;

        // In the CartesianController class declaration:
        double damping_factor_; // For SVD damping

        // PID control variables
        KDL::Vector error_integral = KDL::Vector::Zero(); // For I term
        KDL::Vector last_error = KDL::Vector::Zero();     // For D term
        std::vector<double> ki_ = {0.1, 0.1, 0.1};        // Integral gains
        std::vector<double> kd_ = {0.05, 0.05, 0.05};     // Derivative gains

        // Publishers for Gazebo integration
        std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> velocity_publishers_;

        // Objects for logging
        std::shared_ptr<rclcpp::Logger> logger_;
        rclcpp::Clock::SharedPtr clock_;
        PrintFrequency printFreq;
    };
}

#endif // CARTESIAN_CONTROLLER_HPP
