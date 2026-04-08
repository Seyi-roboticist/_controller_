/**
 * @file safety_validator.hpp
 * @author Seyi R. Afolayan
 * @brief Safety validation interface for ROS 2 ↔ PLC control handoff
 *
 * @details This header defines the safety validation layer that governs the boundary
 * between ROS 2 adaptive control and PLC deterministic safety logic. In a properly
 * architected system, the PLC retains ultimate authority over safety-critical functions
 * (E-stop, light curtains, safety zones, speed monitoring). The ROS 2 layer provides
 * adaptive setpoints and monitoring but must not bypass PLC safety interlocks.
 *
 * This validator enforces that contract:
 *   - All writes to safety-tagged PLC variables are validated before transmission
 *   - E-stop and safety interlock states are monitored and propagated to the ROS 2 side
 *   - Velocity and position commands are range-checked against PLC-configured limits
 *   - The system enters a safe state on communication loss (watchdog)
 *
 * @note Design philosophy: The safety validator does NOT implement safety logic — that
 * belongs in the PLC program where it runs deterministically on certified hardware.
 * This layer ensures that the ROS 2 side cannot inadvertently violate the safety
 * boundaries established by the PLC program. The distinction matters for compliance:
 * the PLC handles SIL-rated safety functions; this layer handles communication integrity.
 *
 * @note This is an interface definition with documentation. Concrete implementations
 * should be validated against specific PLC safety programs and site requirements.
 *
 * @see IEC 62443 (Industrial network security)
 * @see IEC 61508 (Functional safety)
 * @see IEC 62061 / ISO 13849 (Safety of machinery)
 *
 * @date 2025
 * @copyright MIT License
 */

#if !defined(PLC_ROS2_BRIDGE__SAFETY_VALIDATOR_HPP_)
#define PLC_ROS2_BRIDGE__SAFETY_VALIDATOR_HPP_

#include "plc_ros2_bridge/plc_interface.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace plc_ros2_bridge
{
    // ─────────────────────────────────────────────────────────────────────────
    //  Safety State Types
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * @brief Safety states for the ROS 2 ↔ PLC bridge
     *
     * @details These states mirror common PLC safety lifecycle states.
     * The state machine enforces that transitions only occur through
     * validated paths — e.g., you cannot go from ESTOP to OPERATIONAL
     * without passing through RESET_REQUIRED → IDLE.
     */
    enum class SafetyState
    {
        INITIALIZING,    ///< System starting up, no commands allowed
        IDLE,            ///< Connected and safe, waiting for enable
        OPERATIONAL,     ///< Normal operation, commands flow to PLC
        HOLDING,         ///< Temporarily paused (soft stop), resume possible
        ESTOP,           ///< Emergency stop active, all outputs zeroed
        FAULT,           ///< Fault detected, manual intervention required
        RESET_REQUIRED   ///< Fault cleared, awaiting operator reset acknowledgment
    };

    /**
     * @brief Classification of safety-related events
     */
    enum class SafetyEvent
    {
        ESTOP_PRESSED,          ///< Physical or software E-stop activated
        ESTOP_RELEASED,         ///< E-stop released (still requires reset)
        SAFETY_ZONE_VIOLATED,   ///< Robot entered a restricted zone
        SPEED_LIMIT_EXCEEDED,   ///< Commanded velocity exceeds safe limit
        POSITION_LIMIT_EXCEEDED,///< Commanded position outside safe envelope
        WATCHDOG_TIMEOUT,       ///< Communication heartbeat lost
        PLC_FAULT,              ///< PLC reported a fault condition
        OPERATOR_RESET,         ///< Operator acknowledged fault and reset
        ENABLE_REQUESTED,       ///< Transition from IDLE to OPERATIONAL requested
        HOLD_REQUESTED,         ///< Soft stop / pause requested
        RESUME_REQUESTED        ///< Resume from HOLDING state
    };

    /**
     * @brief Defines safe operating limits for a joint or axis
     *
     * @details These limits are typically configured to match or be more
     * restrictive than the corresponding safety limits in the PLC program.
     * The ROS 2 side validates commands against these limits BEFORE sending
     * to the PLC, providing defense-in-depth.
     */
    struct SafetyLimits
    {
        std::string joint_name;

        double min_position = 0.0;      ///< Minimum position [rad or m]
        double max_position = 0.0;      ///< Maximum position [rad or m]
        double max_velocity = 0.0;      ///< Maximum velocity [rad/s or m/s]
        double max_acceleration = 0.0;  ///< Maximum acceleration [rad/s² or m/s²]
        double max_torque = 0.0;        ///< Maximum torque/force [Nm or N]

        /**
         * @brief Check if a commanded position is within safe limits
         */
        bool isPositionSafe(double position) const
        {
            return position >= min_position && position <= max_position;
        }

        /**
         * @brief Check if a commanded velocity is within safe limits
         */
        bool isVelocitySafe(double velocity) const
        {
            return std::abs(velocity) <= max_velocity;
        }
    };

    /**
     * @brief Configuration for the watchdog timer
     *
     * @details The watchdog ensures that communication loss between ROS 2
     * and the PLC triggers a safe stop. Both sides maintain independent
     * watchdogs:
     *   - PLC-side: If the PLC doesn't receive a heartbeat from ROS 2
     *     within the timeout, the PLC program should zero outputs and
     *     enter a safe state autonomously.
     *   - ROS 2-side (this): If the ROS 2 bridge can't confirm PLC
     *     responsiveness, it stops sending commands and reports FAULT.
     */
    struct WatchdogConfig
    {
        std::chrono::milliseconds timeout{500};    ///< Max time without heartbeat
        std::string heartbeat_tag;                 ///< PLC tag for heartbeat counter
        std::string watchdog_status_tag;           ///< PLC tag indicating watchdog OK
        bool enabled = true;
    };

    // ─────────────────────────────────────────────────────────────────────────
    //  Tag Classification for Safety
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * @brief Classifies PLC tags by their safety relevance
     *
     * @details Not all PLC tags are safety-critical. This classification
     * determines the validation rules applied to each tag:
     *   - COMMAND tags get range-checked and state-gated
     *   - STATUS tags are read-only from the ROS 2 side
     *   - SAFETY tags require the system to be in OPERATIONAL state
     *   - DIAGNOSTIC tags have no safety restrictions
     */
    enum class TagSafetyClass
    {
        COMMAND,      ///< Setpoints and commands TO the PLC (validated)
        STATUS,       ///< Feedback and status FROM the PLC (read-only)
        SAFETY_IO,    ///< Safety-rated I/O (E-stop, light curtains)
        DIAGNOSTIC    ///< Non-safety monitoring data
    };

    /**
     * @brief Maps a PLC tag to its safety classification and validation rules
     */
    struct SafetyTagMapping
    {
        std::string tag_name;
        TagSafetyClass safety_class;
        std::string associated_joint;  ///< For COMMAND tags: which joint this controls
        double min_value = -std::numeric_limits<double>::infinity();
        double max_value = std::numeric_limits<double>::infinity();
    };

    // ─────────────────────────────────────────────────────────────────────────
    //  Safety Validator Interface
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * @brief Safety validation layer for the ROS 2 ↔ PLC bridge
     *
     * @details This class sits between the ros2_control hardware interface
     * and the PLC communication interface. Every write command passes through
     * the validator before reaching the PLC interface.
     *
     * Architecture:
     *
     *   ┌──────────────────────┐
     *   │  ros2_control        │  (Cartesian controller, joint trajectory, etc.)
     *   │  Controller Manager  │
     *   └────────┬─────────────┘
     *            │ command interfaces (velocity, position)
     *   ┌────────▼─────────────┐
     *   │  PLC Hardware        │  (ros2_control SystemInterface)
     *   │  Interface           │
     *   └────────┬─────────────┘
     *            │ write requests
     *   ┌────────▼─────────────┐
     *   │  Safety Validator    │  ← THIS LAYER
     *   │  (validates before   │
     *   │   forwarding)        │
     *   └────────┬─────────────┘
     *            │ validated writes
     *   ┌────────▼─────────────┐
     *   │  PLCInterface        │  (EtherNet/IP or OPC UA)
     *   │  (protocol layer)    │
     *   └────────┬─────────────┘
     *            │ network I/O
     *   ┌────────▼─────────────┐
     *   │  PLC                 │  (Allen-Bradley, Siemens, etc.)
     *   │  (safety authority)  │
     *   └──────────────────────┘
     *
     * @note The PLC always retains ultimate safety authority. This validator
     * provides defense-in-depth from the ROS 2 side. A properly configured
     * PLC safety program should independently reject any unsafe commands,
     * but the validator prevents them from being sent in the first place.
     */
    class SafetyValidator
    {
    public:
        using SafetyEventCallback = std::function<void(SafetyEvent, const std::string &)>;
        using SharedPtr = std::shared_ptr<SafetyValidator>;

        SafetyValidator() = default;
        virtual ~SafetyValidator() = default;

        // ── Configuration ───────────────────────────────────────────────

        /**
         * @brief Configure safety limits for all controlled joints
         * @param limits Vector of per-joint safety limits
         */
        virtual void configureLimits(const std::vector<SafetyLimits> &limits) = 0;

        /**
         * @brief Configure the PLC tag safety classifications
         * @param mappings Vector of tag-to-safety-class mappings
         */
        virtual void configureTagMappings(const std::vector<SafetyTagMapping> &mappings) = 0;

        /**
         * @brief Configure the watchdog timer
         * @param config Watchdog configuration
         */
        virtual void configureWatchdog(const WatchdogConfig &config) = 0;

        /**
         * @brief Register a callback for safety events
         * @param callback Function called when safety events occur
         */
        virtual void registerEventCallback(SafetyEventCallback callback) = 0;

        // ── State Machine ───────────────────────────────────────────────

        /**
         * @brief Get the current safety state
         */
        virtual SafetyState getState() const = 0;

        /**
         * @brief Process a safety event and transition state if appropriate
         *
         * @details Valid state transitions:
         *   INITIALIZING → IDLE          (on successful startup)
         *   IDLE → OPERATIONAL           (on ENABLE_REQUESTED)
         *   OPERATIONAL → HOLDING        (on HOLD_REQUESTED)
         *   HOLDING → OPERATIONAL        (on RESUME_REQUESTED)
         *   OPERATIONAL → ESTOP          (on ESTOP_PRESSED)
         *   HOLDING → ESTOP             (on ESTOP_PRESSED)
         *   ESTOP → RESET_REQUIRED       (on ESTOP_RELEASED)
         *   RESET_REQUIRED → IDLE        (on OPERATOR_RESET)
         *   ANY → FAULT                  (on WATCHDOG_TIMEOUT, PLC_FAULT)
         *   FAULT → RESET_REQUIRED       (when fault condition cleared)
         *
         * @param event The safety event to process
         * @return The new safety state after processing
         */
        virtual SafetyState processEvent(SafetyEvent event) = 0;

        // ── Validation ──────────────────────────────────────────────────

        /**
         * @brief Validate a single tag write against safety rules
         *
         * @details Checks:
         *   1. Is the system in a state that allows writes? (OPERATIONAL)
         *   2. Is this tag writable from the ROS 2 side?
         *   3. Is the value within the configured safe range?
         *   4. Is the watchdog still healthy?
         *
         * @param tag_name The PLC tag being written
         * @param value The value to write
         * @return IOResult::SUCCESS if write is permitted, or the specific failure reason
         */
        virtual IOResult validateWrite(const std::string &tag_name, const PLCValue &value) = 0;

        /**
         * @brief Validate a batch of tag writes
         * @param tag_values Map of tag name → value
         * @return Map of tag name → validation result
         */
        virtual std::unordered_map<std::string, IOResult> validateWrites(
            const std::unordered_map<std::string, PLCValue> &tag_values) = 0;

        /**
         * @brief Validate joint velocity commands against safety limits
         *
         * @details This is the primary validation path for the Cartesian controller.
         * Called every control cycle with the computed joint velocities before they
         * are written to PLC command tags.
         *
         * @param joint_names Ordered joint names
         * @param velocities Commanded velocities (same order as joint_names)
         * @return true if all velocities are within safe limits
         */
        virtual bool validateJointVelocities(
            const std::vector<std::string> &joint_names,
            const std::vector<double> &velocities) = 0;

        /**
         * @brief Validate joint position commands against safety limits
         *
         * @param joint_names Ordered joint names
         * @param positions Commanded positions (same order as joint_names)
         * @return true if all positions are within safe limits
         */
        virtual bool validateJointPositions(
            const std::vector<std::string> &joint_names,
            const std::vector<double> &positions) = 0;

        // ── Watchdog ────────────────────────────────────────────────────

        /**
         * @brief Update the watchdog timer (call every control cycle)
         *
         * @details Should be called from the hardware interface's read() method
         * after successfully reading from the PLC. If this is not called within
         * the watchdog timeout period, the validator transitions to FAULT state
         * and blocks all further writes.
         */
        virtual void feedWatchdog() = 0;

        /**
         * @brief Check if the watchdog is healthy
         * @return true if the last feedWatchdog() call was within the timeout
         */
        virtual bool isWatchdogHealthy() const = 0;

        // ── Diagnostics ─────────────────────────────────────────────────

        /**
         * @brief Get a human-readable description of the current safety state
         */
        virtual std::string getStateDescription() const = 0;

        /**
         * @brief Get the number of writes blocked by safety validation
         */
        virtual uint64_t getBlockedWriteCount() const = 0;
    };

}  // namespace plc_ros2_bridge

#endif  // PLC_ROS2_BRIDGE__SAFETY_VALIDATOR_HPP_
