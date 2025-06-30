/**
 * @author Seyi R. Afolayan
 * @date 31-Mar-2025
 *
 * @cite ros2_control_demos
 */

#if !defined(SE3_SENSOR_DRIVER__HARDWARE_INTERFACE_HPP_)
#define SE3_SENSOR_DRIVER__HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>

#include <hardware_interface/handle.hpp>           // interface handles
#include <hardware_interface/hardware_info.hpp>    // hardware config info
#include <hardware_interface/sensor_interface.hpp> // Base class for sensor interfaces
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/macros.hpp>          // For ROS2 C++ macros
#include <rclcpp_lifecycle/state.hpp> // For lifecycle state management

namespace se3_sensor_driver
{
    class SE3SensorHardware : public hardware_interface::SensorInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(SE3SensorHardware) // Figure out what the heck this shared ptr really means

        /* Using the API, I define the following methods. For on_activate and on_deactivated, I followed the example to fulfill the interface contract */

        /// @brief Initialization of the hardware interface from data parsed from the robot's URDF (Virtual Method)
        /// @param hardware_info - [in] structure with data from URDF
        /// @return CallbackReturn::SUCCESS if required data are provided and can be parsed
        /// @return CallbackReturn::ERROR if any error happens or data are missing
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;

        /// @brief Initialize connections to actual hardware, reset state variables, and/or perform setup tasks.
        /// @param previous_state The previous lifecycle state before activation
        /// @return SUCCESS if activation was successful, ERROR otherwise
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

        /// @brief Shutdowns connections to the hardware and stuff
        /// @param previous_state The previous lifecycle state before deactivation
        /// @return SUCCESS if deactivation was successful, ERROR otherwise
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        /// @brief Exports all state interfaces for this hardware interface
        /// @return vectors of state interfaces         
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        /// @brief Reads the current state data from the sensor hardware
        /// @param time -> current time: The time at the start of this control loop iteration 
        /// @param period -> Time since the last read: The measured time taken by the last control loop iteration`
        /// @return OK if successful, ERROR otherwise
        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        /// @brief Unlike the node class, there is no get_logger method. So we need to manually create it. 
        /// @return Logger of the SensorInterface
        rclcpp::Logger get_logger() const { return *logger_; }

        /// @brief Get the clock of the SensorInterface
        /// @return Logger of the Sensor Interface
        rclcpp::Clock::SharedPtr get_clock() const { return clock_; }

    private:
        // Parameters for the SE3 sensor 
        std::string ip_address_;

        // Map to store frame_ids for each sensor 
        std::map<std::string, std::string> sensor_frame_ids_;

        // Socket communication variables 
        int sockfd_; 
        bool connected_;

        // Reconnection parameters
        int reconnect_attempts_;
        int max_reconnect_attempts_;
        int reconnect_delay_ms_;

        // Socket communication methods 
        bool connectToServer();

        // Objects for logging 
        std::shared_ptr<rclcpp::Logger> logger_; 
        rclcpp::Clock::SharedPtr clock_; 

        // Store the sensor states (position and orientation)
        // [x y z qx qy qz qw]
        std::vector<double> hw_sensor_states_;
    };

}

#endif // SE3_SENSOR_DRIVER__HARDWARE_INTERFACE_HPP_
