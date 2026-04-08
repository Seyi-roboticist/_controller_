/**
 * @file plc_hardware_interface.hpp
 * @author Seyi R. Afolayan
 * @brief ros2_control SystemInterface for PLC-bridged robot control
 *
 * @details This hardware interface implements the ros2_control SystemInterface to bridge
 * ROS 2 controllers with PLC-governed industrial robots. It replaces the direct
 * hardware communication layer (e.g., UR driver, custom EtherCAT) with a PLC
 * intermediary, reflecting the architecture used in production manufacturing cells
 * where the PLC is the safety-rated controller of record.
 *
 * Data flow:
 *   ROS 2 Controller → [command interfaces] → PLCHardwareInterface
 *     → SafetyValidator (validates) → PLCInterface (EtherNet/IP or OPC UA) → PLC
 *
 *   PLC → PLCInterface (reads) → PLCHardwareInterface → [state interfaces]
 *     → ROS 2 Controller
 *
 * This architecture preserves the PLC's role as the deterministic safety controller
 * while enabling ROS 2's adaptive control capabilities (path planning, force control,
 * visual servoing) to provide setpoints.
 *
 * @date 2025
 * @copyright MIT License
 */

#if !defined(PLC_ROS2_BRIDGE__PLC_HARDWARE_INTERFACE_HPP_)
#define PLC_ROS2_BRIDGE__PLC_HARDWARE_INTERFACE_HPP_

#include "plc_ros2_bridge/plc_interface.hpp"
#include "plc_ros2_bridge/safety_validator.hpp"
#include "plc_ros2_bridge/tag_registry.hpp"

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <memory>
#include <string>
#include <vector>

namespace plc_ros2_bridge
{
    /**
     * @brief ros2_control SystemInterface bridging controllers to a PLC
     *
     * @details This is the integration point where the vendor-neutral PLC
     * architecture meets the ros2_control framework. It exposes standard
     * command/state interfaces (position, velocity, effort) that any
     * ros2_control-compatible controller can use, while internally
     * translating those to PLC tag reads/writes through the safety validator.
     *
     * URDF configuration (in the ros2_control xacro):
     * @code
     * <ros2_control name="PLCBridge" type="system">
     *   <hardware>
     *     <plugin>plc_ros2_bridge/PLCHardwareInterface</plugin>
     *     <param name="protocol">ethernet_ip</param>
     *     <param name="ip_address">192.168.1.10</param>
     *     <param name="slot">0</param>
     *     <param name="tag_mapping_file">config/tag_mapping.yaml</param>
     *     <param name="safety_config_file">config/safety_config.yaml</param>
     *   </hardware>
     *   <joint name="shoulder_pan_joint">
     *     <command_interface name="velocity"/>
     *     <state_interface name="position"/>
     *     <state_interface name="velocity"/>
     *   </joint>
     *   <!-- ... more joints ... -->
     * </ros2_control>
     * @endcode
     */
    class PLCHardwareInterface : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(PLCHardwareInterface)

        PLCHardwareInterface();
        ~PLCHardwareInterface() override;

        // ── Lifecycle ────────────────────────────────────────────────────

        /**
         * @brief Initialize from URDF hardware configuration
         *
         * @details Parses hardware parameters from the URDF to determine:
         *   - Which PLC protocol to use (ethernet_ip or opcua)
         *   - PLC connection parameters (IP, port, slot, etc.)
         *   - Tag mapping file location
         *   - Safety configuration file location
         *
         * Also initializes the tag registry and safety validator.
         */
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo &hardware_info) override;

        /**
         * @brief Establish connection to the PLC and validate tag map
         *
         * @details During activation:
         *   1. Creates the appropriate PLCInterface (EtherNet/IP or OPC UA)
         *   2. Connects to the PLC
         *   3. Validates that all required tags exist in the PLC program
         *   4. Initializes the safety validator
         *   5. Starts the watchdog
         */
        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        /**
         * @brief Gracefully disconnect from the PLC
         *
         * @details Zeros all command outputs, stops the watchdog, and
         * disconnects from the PLC.
         */
        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        // ── Interface Export ─────────────────────────────────────────────

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        // ── Cyclic I/O ──────────────────────────────────────────────────

        /**
         * @brief Read joint states from the PLC
         *
         * @details Called by the controller manager on every control cycle.
         * Performs a batch read of all feedback tags from the PLC, applies
         * unit conversions via the tag registry, and updates the state
         * interface values.
         *
         * Also reads safety-relevant tags (E-stop, faults) and feeds
         * the watchdog timer.
         */
        hardware_interface::return_type read(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        /**
         * @brief Write joint commands to the PLC
         *
         * @details Called by the controller manager on every control cycle.
         * Takes the current command interface values (set by the active
         * controller), validates them through the safety layer, applies
         * unit conversions, and performs a batch write to the PLC.
         *
         * If the safety validator blocks any write, the entire batch is
         * rejected and zero commands are sent to maintain safe state.
         */
        hardware_interface::return_type write(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        // ── Logging ─────────────────────────────────────────────────────

        rclcpp::Logger get_logger() const { return *logger_; }
        rclcpp::Clock::SharedPtr get_clock() const { return clock_; }

    private:
        /**
         * @brief Factory method to create the appropriate PLCInterface
         * @param protocol "ethernet_ip" or "opcua"
         * @return Configured but not connected PLCInterface
         */
        PLCInterface::UniquePtr createPLCInterface(const std::string &protocol);

        /**
         * @brief Create the safety validator implementation
         */
        SafetyValidator::SharedPtr createSafetyValidator();

        // ── Components ──────────────────────────────────────────────────

        PLCInterface::UniquePtr plc_interface_;       ///< Protocol-specific PLC communication
        SafetyValidator::SharedPtr safety_validator_;  ///< Safety validation layer
        TagRegistry tag_registry_;                     ///< ROS 2 ↔ PLC tag mappings

        // ── Configuration ───────────────────────────────────────────────

        std::string protocol_;            ///< "ethernet_ip" or "opcua"
        std::string ip_address_;
        std::string tag_mapping_file_;
        std::string safety_config_file_;
        std::unordered_map<std::string, std::string> plc_params_;  ///< Protocol-specific params

        // ── Interface Data ──────────────────────────────────────────────

        std::vector<std::string> joint_names_;

        // State interface storage (read from PLC)
        std::vector<double> hw_positions_;
        std::vector<double> hw_velocities_;
        std::vector<double> hw_efforts_;

        // Command interface storage (written to PLC)
        std::vector<double> hw_cmd_positions_;
        std::vector<double> hw_cmd_velocities_;
        std::vector<double> hw_cmd_efforts_;

        // ── Logging ─────────────────────────────────────────────────────

        std::shared_ptr<rclcpp::Logger> logger_;
        rclcpp::Clock::SharedPtr clock_;
    };

}  // namespace plc_ros2_bridge

#endif  // PLC_ROS2_BRIDGE__PLC_HARDWARE_INTERFACE_HPP_
