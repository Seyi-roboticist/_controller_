/**
 * @file plc_interface.hpp
 * @author Seyi R. Afolayan
 * @brief Vendor-neutral abstract interface for PLC communication in ROS 2 control systems
 *
 * @details This header defines the core abstraction layer that enables ROS 2-based adaptive
 * robotics systems to interface with legacy PLC-governed industrial control systems without
 * vendor lock-in. The interface establishes a common contract for reading and writing PLC
 * tags, managing connection lifecycles, and enforcing safety-critical handoff semantics.
 *
 * Protocol implementations (EtherNet/IP, OPC UA, Modbus TCP, etc.) inherit from this
 * interface, allowing system integrators to swap communication backends without modifying
 * the control architecture above this layer.
 *
 * @note Design rationale: Industrial control systems in U.S. manufacturing are dominated by
 * proprietary PLC ecosystems (Allen-Bradley/Rockwell, Siemens, Mitsubishi, Omron). Each uses
 * different communication protocols and tag addressing schemes. This interface abstracts those
 * differences so that a single ROS 2 control architecture can deploy across heterogeneous
 * factory floors — the vendor-neutral integration pattern central to this project.
 *
 * @date 2025
 * @copyright MIT License
 */

#if !defined(PLC_ROS2_BRIDGE__PLC_INTERFACE_HPP_)
#define PLC_ROS2_BRIDGE__PLC_INTERFACE_HPP_

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

namespace plc_ros2_bridge
{
    // ─────────────────────────────────────────────────────────────────────────
    //  Tag Value Types
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * @brief Variant type representing the union of PLC-native data types
     *
     * @details PLC systems use typed tags/registers. This variant covers the
     * common data types across major PLC platforms:
     *   - BOOL  → bool
     *   - SINT  → int8_t
     *   - INT   → int16_t
     *   - DINT  → int32_t
     *   - REAL  → float
     *   - LREAL → double
     *
     * Allen-Bradley uses CIP data types; Siemens uses S7 types; OPC UA has
     * its own type system. This variant normalizes all of them into C++ types.
     */
    using PLCValue = std::variant<bool, int8_t, int16_t, int32_t, float, double>;

    // ─────────────────────────────────────────────────────────────────────────
    //  Connection & Status Types
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * @brief Connection states for the PLC communication link
     */
    enum class ConnectionState
    {
        DISCONNECTED,  ///< No active connection
        CONNECTING,    ///< Connection attempt in progress
        CONNECTED,     ///< Connection established, ready for I/O
        ERROR,         ///< Connection failed or lost
        RECONNECTING   ///< Automatic reconnection in progress
    };

    /**
     * @brief Result of a read or write operation
     */
    enum class IOResult
    {
        SUCCESS,              ///< Operation completed successfully
        TIMEOUT,              ///< Operation timed out
        TAG_NOT_FOUND,        ///< Requested tag does not exist on the PLC
        TYPE_MISMATCH,        ///< Value type does not match tag type
        PERMISSION_DENIED,    ///< Insufficient access rights
        CONNECTION_LOST,      ///< Connection dropped during operation
        SAFETY_INTERLOCK,     ///< Operation blocked by safety validator
        PROTOCOL_ERROR        ///< Protocol-level error
    };

    /**
     * @brief Metadata about a PLC tag, used for validation and diagnostics
     */
    struct TagInfo
    {
        std::string name;           ///< Fully qualified tag name (e.g., "Program:Main.JointCmd[0]")
        std::string data_type;      ///< PLC-native type name (e.g., "REAL", "DINT", "BOOL")
        bool is_writable = true;    ///< Whether the tag accepts write operations
        bool is_safety_tag = false; ///< Whether the tag is part of the safety I/O map
        std::string description;    ///< Human-readable tag description
    };

    /**
     * @brief Diagnostics snapshot for connection health monitoring
     */
    struct ConnectionDiagnostics
    {
        ConnectionState state = ConnectionState::DISCONNECTED;
        uint64_t successful_reads = 0;
        uint64_t failed_reads = 0;
        uint64_t successful_writes = 0;
        uint64_t failed_writes = 0;
        double avg_round_trip_ms = 0.0;     ///< Moving average of I/O round-trip time
        double max_round_trip_ms = 0.0;     ///< Worst-case round-trip observed
        uint32_t reconnect_count = 0;       ///< Number of reconnections since activation
        std::string last_error_message;     ///< Most recent error description
    };

    // ─────────────────────────────────────────────────────────────────────────
    //  Abstract PLC Interface
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * @brief Abstract interface for vendor-neutral PLC communication
     *
     * @details This is the core abstraction enabling the ROS 2 ↔ PLC bridge.
     * Concrete implementations handle protocol-specific details:
     *
     *   - EtherNetIPInterface  → Allen-Bradley / Rockwell Automation
     *   - OPCUAInterface       → Vendor-neutral (Siemens, Beckhoff, B&R, etc.)
     *   - ModbusTCPInterface   → Legacy systems (future implementation)
     *
     * The interface enforces a clear lifecycle:
     *   1. configure()  — Set connection parameters
     *   2. connect()    — Establish communication link
     *   3. read/write   — Cyclic I/O operations
     *   4. disconnect() — Graceful shutdown
     *
     * Thread safety: Implementations must be safe for concurrent read/write
     * calls from the ros2_control update loop, which runs in a real-time
     * context on the controller manager's thread.
     */
    class PLCInterface
    {
    public:
        using SharedPtr = std::shared_ptr<PLCInterface>;
        using UniquePtr = std::unique_ptr<PLCInterface>;

        PLCInterface() = default;
        virtual ~PLCInterface() = default;

        // Non-copyable, movable
        PLCInterface(const PLCInterface &) = delete;
        PLCInterface &operator=(const PLCInterface &) = delete;
        PLCInterface(PLCInterface &&) = default;
        PLCInterface &operator=(PLCInterface &&) = default;

        // ── Lifecycle ────────────────────────────────────────────────────

        /**
         * @brief Configure the connection parameters
         *
         * @param params Key-value configuration map. Common keys:
         *   - "ip_address"    : PLC IP (e.g., "192.168.1.10")
         *   - "port"          : Communication port
         *   - "timeout_ms"    : I/O timeout in milliseconds
         *   - "slot"          : Rack/slot for Allen-Bradley
         *   - "endpoint_url"  : OPC UA endpoint URL
         *   - "namespace_uri" : OPC UA namespace
         *
         * @return true if configuration is valid, false otherwise
         */
        virtual bool configure(const std::unordered_map<std::string, std::string> &params) = 0;

        /**
         * @brief Establish connection to the PLC
         * @return true if connection succeeds
         */
        virtual bool connect() = 0;

        /**
         * @brief Gracefully close the PLC connection
         * @return true if disconnection succeeds
         */
        virtual bool disconnect() = 0;

        /**
         * @brief Check whether the connection is active
         * @return true if connected and ready for I/O
         */
        virtual bool isConnected() const = 0;

        /**
         * @brief Attempt to re-establish a lost connection
         * @return true if reconnection succeeds
         */
        virtual bool reconnect() = 0;

        // ── Tag I/O ─────────────────────────────────────────────────────

        /**
         * @brief Read a single tag value from the PLC
         *
         * @param tag_name Fully qualified tag name
         * @param[out] value The read value
         * @return IOResult indicating success or failure mode
         */
        virtual IOResult readTag(const std::string &tag_name, PLCValue &value) = 0;

        /**
         * @brief Write a single tag value to the PLC
         *
         * @param tag_name Fully qualified tag name
         * @param value The value to write
         * @return IOResult indicating success or failure mode
         *
         * @note Implementations must validate that the tag is writable and
         * that the value type matches the PLC tag type before writing.
         * Safety-tagged values must be validated by the safety layer.
         */
        virtual IOResult writeTag(const std::string &tag_name, const PLCValue &value) = 0;

        /**
         * @brief Batch-read multiple tags in a single transaction
         *
         * @details Batch operations reduce network round-trips, which is
         * critical for meeting real-time control deadlines. EtherNet/IP
         * supports this natively via CIP Multiple Service Packet;
         * OPC UA supports it via ReadRequest with multiple ReadValueIds.
         *
         * @param tag_names List of tag names to read
         * @param[out] values Map of tag name → value for successful reads
         * @return Map of tag name → IOResult for each tag
         */
        virtual std::unordered_map<std::string, IOResult> readTags(
            const std::vector<std::string> &tag_names,
            std::unordered_map<std::string, PLCValue> &values) = 0;

        /**
         * @brief Batch-write multiple tags in a single transaction
         *
         * @param tag_values Map of tag name → value to write
         * @return Map of tag name → IOResult for each tag
         */
        virtual std::unordered_map<std::string, IOResult> writeTags(
            const std::unordered_map<std::string, PLCValue> &tag_values) = 0;

        // ── Tag Discovery & Validation ──────────────────────────────────

        /**
         * @brief Query metadata for a specific tag
         *
         * @param tag_name Fully qualified tag name
         * @param[out] info Tag metadata
         * @return true if tag exists, false otherwise
         */
        virtual bool getTagInfo(const std::string &tag_name, TagInfo &info) = 0;

        /**
         * @brief Verify that a set of required tags exist and are accessible
         *
         * @details Called during configuration to fail fast if the PLC program
         * doesn't have the expected tags. This catches mismatches between
         * the ROS 2 controller configuration and the PLC program at startup
         * rather than during operation.
         *
         * @param required_tags List of tag names that must exist
         * @return Map of tag name → bool indicating presence
         */
        virtual std::unordered_map<std::string, bool> validateTags(
            const std::vector<std::string> &required_tags) = 0;

        // ── Diagnostics ─────────────────────────────────────────────────

        /**
         * @brief Get current connection diagnostics
         * @return Diagnostics snapshot
         */
        virtual ConnectionDiagnostics getDiagnostics() const = 0;

        /**
         * @brief Get the protocol identifier string
         * @return Human-readable protocol name (e.g., "EtherNet/IP", "OPC UA")
         */
        virtual std::string getProtocolName() const = 0;
    };

}  // namespace plc_ros2_bridge

#endif  // PLC_ROS2_BRIDGE__PLC_INTERFACE_HPP_
