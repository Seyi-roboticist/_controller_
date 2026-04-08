/**
 * @file tag_registry.hpp
 * @author Seyi R. Afolayan
 * @brief Tag mapping registry between ROS 2 control interfaces and PLC tag addresses
 *
 * @details The tag registry solves the naming translation problem: ros2_control uses
 * interface names like "shoulder_pan_joint/velocity" while PLCs use tag names like
 * "Program:Main.ROS_JointVelCmd[0]". This registry maintains the bidirectional
 * mapping and handles array indexing, structure member access, and protocol-specific
 * address translation.
 *
 * The registry is populated from a YAML configuration file, enabling integrators to
 * adapt the bridge to any PLC program without modifying source code. This is a key
 * enabler of the vendor-neutral architecture: the same bridge code works with
 * different PLC programs by changing only the tag mapping configuration.
 *
 * @date 2025
 * @copyright MIT License
 */

#if !defined(PLC_ROS2_BRIDGE__TAG_REGISTRY_HPP_)
#define PLC_ROS2_BRIDGE__TAG_REGISTRY_HPP_

#include "plc_ros2_bridge/plc_interface.hpp"
#include "plc_ros2_bridge/safety_validator.hpp"

#include <string>
#include <unordered_map>
#include <vector>

namespace plc_ros2_bridge
{
    /**
     * @brief Defines the direction of data flow for a tag mapping
     */
    enum class TagDirection
    {
        TO_PLC,     ///< ROS 2 writes, PLC reads (command)
        FROM_PLC,   ///< PLC writes, ROS 2 reads (feedback/status)
        BIDIRECTIONAL  ///< Both directions (rare, use with caution)
    };

    /**
     * @brief A single mapping entry between a ROS 2 interface and a PLC tag
     */
    struct TagMapping
    {
        // ROS 2 side
        std::string ros2_interface;      ///< e.g., "shoulder_pan_joint/velocity"
        std::string joint_name;          ///< e.g., "shoulder_pan_joint"
        std::string interface_type;      ///< e.g., "velocity", "position", "effort"

        // PLC side
        std::string plc_tag;             ///< e.g., "Program:Main.ROS_JointVelCmd[0]"

        // Metadata
        TagDirection direction = TagDirection::TO_PLC;
        TagSafetyClass safety_class = TagSafetyClass::COMMAND;

        // Value transformation (linear: plc_value = scale * ros2_value + offset)
        double scale = 1.0;             ///< Unit conversion scale factor
        double offset = 0.0;            ///< Unit conversion offset

        /**
         * @brief Apply the forward transformation (ROS 2 → PLC)
         */
        double toPLC(double ros2_value) const
        {
            return scale * ros2_value + offset;
        }

        /**
         * @brief Apply the inverse transformation (PLC → ROS 2)
         */
        double fromPLC(double plc_value) const
        {
            return (plc_value - offset) / scale;
        }
    };

    /**
     * @brief Registry managing ROS 2 ↔ PLC tag mappings
     *
     * @details Provides lookup in both directions:
     *   - Given a ros2_control interface name → get the PLC tag
     *   - Given a PLC tag name → get the ros2_control interface
     *
     * Also provides grouped access for batch operations:
     *   - All command tags (for batch write in the update loop)
     *   - All feedback tags (for batch read in the read loop)
     *   - All safety tags (for safety state monitoring)
     */
    class TagRegistry
    {
    public:
        TagRegistry() = default;
        ~TagRegistry() = default;

        /**
         * @brief Add a tag mapping to the registry
         */
        void addMapping(const TagMapping &mapping);

        /**
         * @brief Load mappings from a YAML configuration file
         *
         * Expected YAML structure:
         * @code
         * tag_mappings:
         *   - ros2_interface: "shoulder_pan_joint/velocity"
         *     plc_tag: "Program:Main.ROS_JointVelCmd[0]"
         *     direction: "to_plc"
         *     safety_class: "command"
         *     scale: 1.0
         *     offset: 0.0
         *   - ros2_interface: "shoulder_pan_joint/position"
         *     plc_tag: "Program:Main.ROS_JointPosFb[0]"
         *     direction: "from_plc"
         *     safety_class: "status"
         *
         * safety_tags:
         *   - plc_tag: "Safety.EStop_Active"
         *     safety_class: "safety_io"
         *   - plc_tag: "Safety.ROS_Watchdog_OK"
         *     safety_class: "safety_io"
         *
         * watchdog:
         *   heartbeat_tag: "Program:Main.ROS_Heartbeat"
         *   watchdog_status_tag: "Safety.ROS_Watchdog_OK"
         *   timeout_ms: 500
         * @endcode
         *
         * @param filepath Path to the YAML file
         * @return true if loading succeeds
         */
        bool loadFromYAML(const std::string &filepath);

        // ── Lookup ──────────────────────────────────────────────────────

        /**
         * @brief Get the PLC tag for a ROS 2 interface
         * @return Pointer to mapping, or nullptr if not found
         */
        const TagMapping *findByROS2Interface(const std::string &ros2_interface) const;

        /**
         * @brief Get the ROS 2 interface for a PLC tag
         * @return Pointer to mapping, or nullptr if not found
         */
        const TagMapping *findByPLCTag(const std::string &plc_tag) const;

        // ── Grouped Access ──────────────────────────────────────────────

        /**
         * @brief Get all command tags (direction == TO_PLC)
         */
        std::vector<const TagMapping *> getCommandTags() const;

        /**
         * @brief Get all feedback tags (direction == FROM_PLC)
         */
        std::vector<const TagMapping *> getFeedbackTags() const;

        /**
         * @brief Get all safety-classified tags
         */
        std::vector<const TagMapping *> getSafetyTags() const;

        /**
         * @brief Get all PLC tag names for batch operations
         */
        std::vector<std::string> getAllPLCTagNames() const;

        /**
         * @brief Get the number of registered mappings
         */
        size_t size() const { return mappings_.size(); }

        /**
         * @brief Get the watchdog configuration loaded from YAML
         */
        const WatchdogConfig &getWatchdogConfig() const { return watchdog_config_; }

    private:
        std::vector<TagMapping> mappings_;
        std::unordered_map<std::string, size_t> ros2_index_;  ///< ros2_interface → index
        std::unordered_map<std::string, size_t> plc_index_;   ///< plc_tag → index
        WatchdogConfig watchdog_config_;
    };

}  // namespace plc_ros2_bridge

#endif  // PLC_ROS2_BRIDGE__TAG_REGISTRY_HPP_
