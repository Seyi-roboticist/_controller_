/**
 * @file opcua_interface.hpp
 * @author Seyi R. Afolayan
 * @brief OPC UA implementation of the vendor-neutral PLC interface
 *
 * @details OPC UA (Open Platform Communications Unified Architecture) is a
 * platform-independent, service-oriented architecture for industrial communication.
 * Unlike EtherNet/IP (Allen-Bradley-centric) or PROFINET (Siemens-centric), OPC UA
 * is truly vendor-neutral and supported by virtually all modern PLC platforms:
 *
 *   - Siemens S7-1500 (native OPC UA server)
 *   - Beckhoff TwinCAT 3
 *   - B&R Automation Runtime
 *   - Allen-Bradley (via FactoryTalk Linx Gateway)
 *   - Omron NX/NJ series
 *   - CODESYS-based controllers
 *
 * This implementation uses the OPC UA Binary protocol (opc.tcp://) for direct
 * PLC communication without requiring gateway software.
 *
 * @note This implementation exists alongside EtherNetIPInterface to demonstrate
 * that the abstract PLCInterface enables genuine vendor neutrality — the same
 * ROS 2 control architecture can deploy on an Allen-Bradley floor (via EtherNet/IP)
 * or a Siemens/Beckhoff floor (via OPC UA) by swapping only the communication backend.
 *
 * @date 2025
 * @copyright MIT License
 */

#if !defined(PLC_ROS2_BRIDGE__OPCUA_INTERFACE_HPP_)
#define PLC_ROS2_BRIDGE__OPCUA_INTERFACE_HPP_

#include "plc_ros2_bridge/plc_interface.hpp"

#include <atomic>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace plc_ros2_bridge
{
    // ─────────────────────────────────────────────────────────────────────────
    //  OPC UA Constants
    // ─────────────────────────────────────────────────────────────────────────

    namespace opcua
    {
        // OPC UA data type NodeIds (from the OPC UA specification Part 6)
        constexpr uint32_t TYPE_BOOLEAN = 1;
        constexpr uint32_t TYPE_SBYTE = 2;
        constexpr uint32_t TYPE_INT16 = 4;
        constexpr uint32_t TYPE_INT32 = 6;
        constexpr uint32_t TYPE_FLOAT = 10;
        constexpr uint32_t TYPE_DOUBLE = 11;

        // Default connection parameters
        constexpr uint16_t DEFAULT_PORT = 4840;
        constexpr uint32_t DEFAULT_TIMEOUT_MS = 2000;

        // OPC UA service result codes (StatusCodes)
        constexpr uint32_t STATUS_GOOD = 0x00000000;
        constexpr uint32_t STATUS_BAD_NODE_ID_UNKNOWN = 0x80340000;
        constexpr uint32_t STATUS_BAD_TYPE_MISMATCH = 0x80740000;
        constexpr uint32_t STATUS_BAD_NOT_WRITABLE = 0x803B0000;
        constexpr uint32_t STATUS_BAD_TIMEOUT = 0x800A0000;
    }  // namespace opcua

    // ─────────────────────────────────────────────────────────────────────────
    //  OPC UA Node Addressing
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * @brief Represents an OPC UA NodeId for addressing PLC variables
     *
     * @details OPC UA uses NodeIds to address all objects in the server's
     * address space. PLC variables are typically exposed as Variable nodes
     * with NodeIds in a vendor-specific namespace. The NodeId can be:
     *   - Numeric (ns=2;i=12345)
     *   - String  (ns=2;s="PLC1.Robot.JointCmd[0]")
     *   - GUID or Opaque (less common for PLC variables)
     */
    struct OPCUANodeId
    {
        uint16_t namespace_index = 0;
        std::string identifier;  ///< String identifier or numeric ID as string
        bool is_numeric = false;

        /**
         * @brief Parse a NodeId string like "ns=2;s=Robot.JointCmd[0]"
         */
        static OPCUANodeId fromString(const std::string &node_id_str);

        /**
         * @brief Convert to display string
         */
        std::string toString() const;
    };

    // ─────────────────────────────────────────────────────────────────────────
    //  OPC UA Interface Implementation
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * @brief OPC UA implementation of PLCInterface
     *
     * @details Connects to any OPC UA-enabled PLC or industrial controller.
     * Uses the OPC UA Binary protocol for direct communication.
     *
     * Configuration parameters:
     *   - "endpoint_url"    : OPC UA endpoint (e.g., "opc.tcp://192.168.1.10:4840")
     *   - "namespace_uri"   : Application namespace URI for tag resolution (optional)
     *   - "namespace_index" : Default namespace index, default 2 (optional)
     *   - "timeout_ms"      : I/O timeout in milliseconds, default 2000 (optional)
     *   - "security_mode"   : "None", "Sign", or "SignAndEncrypt" (optional, default "None")
     *   - "username"        : Authentication username (optional)
     *   - "password"        : Authentication password (optional)
     *
     * Tag naming convention:
     *   Tags are addressed using OPC UA NodeId strings. For convenience, this
     *   implementation supports shorthand: if a tag name doesn't contain "ns=",
     *   it's treated as a string identifier in the default namespace.
     *   e.g., "Robot.JointCmd[0]" → NodeId(ns=default_ns, s="Robot.JointCmd[0]")
     *
     * Connection lifecycle:
     *   1. Open SecureChannel
     *   2. Create Session
     *   3. Activate Session (with optional authentication)
     *   4. Read/Write/Browse service requests
     *   5. Close Session
     *   6. Close SecureChannel
     */
    class OPCUAInterface : public PLCInterface
    {
    public:
        OPCUAInterface();
        ~OPCUAInterface() override;

        // ── Lifecycle ────────────────────────────────────────────────────

        bool configure(const std::unordered_map<std::string, std::string> &params) override;
        bool connect() override;
        bool disconnect() override;
        bool isConnected() const override;
        bool reconnect() override;

        // ── Tag I/O ─────────────────────────────────────────────────────

        IOResult readTag(const std::string &tag_name, PLCValue &value) override;
        IOResult writeTag(const std::string &tag_name, const PLCValue &value) override;

        std::unordered_map<std::string, IOResult> readTags(
            const std::vector<std::string> &tag_names,
            std::unordered_map<std::string, PLCValue> &values) override;

        std::unordered_map<std::string, IOResult> writeTags(
            const std::unordered_map<std::string, PLCValue> &tag_values) override;

        // ── Tag Discovery & Validation ──────────────────────────────────

        bool getTagInfo(const std::string &tag_name, TagInfo &info) override;

        std::unordered_map<std::string, bool> validateTags(
            const std::vector<std::string> &required_tags) override;

        // ── Diagnostics ─────────────────────────────────────────────────

        ConnectionDiagnostics getDiagnostics() const override;
        std::string getProtocolName() const override { return "OPC UA"; }

    private:
        // ── OPC UA Protocol Methods ─────────────────────────────────────

        /**
         * @brief Open a SecureChannel with the OPC UA server
         */
        bool openSecureChannel();

        /**
         * @brief Create and activate a session
         */
        bool createSession();

        /**
         * @brief Resolve a tag name to an OPC UA NodeId
         *
         * @details If the tag name contains "ns=", it's parsed as a full NodeId.
         * Otherwise, it's treated as a string identifier in the default namespace.
         */
        OPCUANodeId resolveTagName(const std::string &tag_name) const;

        /**
         * @brief Map an OPC UA StatusCode to an IOResult
         */
        IOResult mapStatusCode(uint32_t status_code) const;

        /**
         * @brief Convert between PLCValue and OPC UA Variant encoding
         */
        bool serializeVariant(const PLCValue &value, std::vector<uint8_t> &data, uint32_t &type_id);
        bool deserializeVariant(const std::vector<uint8_t> &data, uint32_t type_id, PLCValue &value);

        // ── Connection State ────────────────────────────────────────────

        std::string endpoint_url_;
        uint16_t default_namespace_index_ = 2;
        std::string namespace_uri_;
        uint32_t timeout_ms_ = opcua::DEFAULT_TIMEOUT_MS;
        std::string security_mode_ = "None";
        std::string username_;
        std::string password_;

        int socket_fd_ = -1;
        uint32_t secure_channel_id_ = 0;
        uint32_t session_id_ = 0;
        std::atomic<bool> connected_{false};

        mutable std::mutex io_mutex_;
        mutable std::mutex diagnostics_mutex_;

        // ── Diagnostics State ───────────────────────────────────────────

        ConnectionDiagnostics diagnostics_;
        std::unordered_map<std::string, TagInfo> tag_cache_;
    };

}  // namespace plc_ros2_bridge

#endif  // PLC_ROS2_BRIDGE__OPCUA_INTERFACE_HPP_
