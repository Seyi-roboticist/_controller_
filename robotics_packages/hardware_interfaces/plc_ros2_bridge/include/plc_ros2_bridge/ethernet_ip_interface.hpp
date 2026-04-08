/**
 * @file ethernet_ip_interface.hpp
 * @author Seyi R. Afolayan
 * @brief EtherNet/IP (CIP) implementation of the vendor-neutral PLC interface
 *
 * @details This class implements PLCInterface for Allen-Bradley / Rockwell Automation
 * controllers that communicate via EtherNet/IP (IEEE 802.3 + CIP protocol).
 *
 * EtherNet/IP is the dominant industrial Ethernet protocol in North American manufacturing,
 * used by the ControlLogix, CompactLogix, and Micro800 families. It uses the Common
 * Industrial Protocol (CIP) over TCP/UDP for both explicit (configuration) and implicit
 * (real-time I/O) messaging.
 *
 * This implementation targets explicit messaging for tag-based read/write operations,
 * which is appropriate for supervisory control where the PLC retains real-time authority
 * and the ROS 2 layer provides adaptive setpoints and monitoring.
 *
 * @note Protocol specifics:
 *   - TCP port 44818 for explicit messaging (encapsulation)
 *   - CIP service codes: Read Tag (0x4C), Write Tag (0x4D), Multiple Service Packet (0x0A)
 *   - Tag paths use symbolic addressing (e.g., "Program:MainProgram.JointPositionCmd[0]")
 *   - Session registration is required before any CIP services
 *
 * @date 2025
 * @copyright MIT License
 */

#if !defined(PLC_ROS2_BRIDGE__ETHERNET_IP_INTERFACE_HPP_)
#define PLC_ROS2_BRIDGE__ETHERNET_IP_INTERFACE_HPP_

#include "plc_ros2_bridge/plc_interface.hpp"

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace plc_ros2_bridge
{
    // ─────────────────────────────────────────────────────────────────────────
    //  CIP Protocol Constants
    // ─────────────────────────────────────────────────────────────────────────

    namespace cip
    {
        // EtherNet/IP encapsulation commands
        constexpr uint16_t CMD_REGISTER_SESSION = 0x0065;
        constexpr uint16_t CMD_UNREGISTER_SESSION = 0x0066;
        constexpr uint16_t CMD_SEND_RR_DATA = 0x006F;  // Send Request/Reply Data

        // CIP service codes
        constexpr uint8_t SVC_READ_TAG = 0x4C;
        constexpr uint8_t SVC_WRITE_TAG = 0x4D;
        constexpr uint8_t SVC_READ_TAG_FRAGMENTED = 0x52;
        constexpr uint8_t SVC_WRITE_TAG_FRAGMENTED = 0x53;
        constexpr uint8_t SVC_MULTIPLE_SERVICE_PACKET = 0x0A;

        // CIP data types (used in tag read/write)
        constexpr uint16_t TYPE_BOOL = 0x00C1;
        constexpr uint16_t TYPE_SINT = 0x00C2;
        constexpr uint16_t TYPE_INT = 0x00C3;
        constexpr uint16_t TYPE_DINT = 0x00C4;
        constexpr uint16_t TYPE_REAL = 0x00CA;
        constexpr uint16_t TYPE_LREAL = 0x00CB;

        // Default communication parameters
        constexpr uint16_t DEFAULT_PORT = 44818;
        constexpr uint32_t DEFAULT_TIMEOUT_MS = 1000;
        constexpr uint8_t DEFAULT_SLOT = 0;

        /**
         * @brief Map CIP type code to byte size
         */
        inline size_t typeSize(uint16_t type_code)
        {
            switch (type_code)
            {
            case TYPE_BOOL:
            case TYPE_SINT:
                return 1;
            case TYPE_INT:
                return 2;
            case TYPE_DINT:
                return 4;
            case TYPE_REAL:
                return 4;
            case TYPE_LREAL:
                return 8;
            default:
                return 0;
            }
        }
    }  // namespace cip

    // ─────────────────────────────────────────────────────────────────────────
    //  EtherNet/IP Interface Implementation
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * @brief EtherNet/IP (CIP) implementation of PLCInterface
     *
     * @details Connects to Allen-Bradley ControlLogix / CompactLogix controllers
     * over EtherNet/IP using explicit messaging for tag read/write operations.
     *
     * Configuration parameters:
     *   - "ip_address"  : PLC IP address (required)
     *   - "slot"        : Processor slot number, default 0 (optional)
     *   - "port"        : TCP port, default 44818 (optional)
     *   - "timeout_ms"  : I/O timeout in milliseconds, default 1000 (optional)
     *   - "path"        : CIP routing path for multi-hop, e.g., "1/0" (optional)
     *
     * Connection lifecycle:
     *   1. TCP connection to port 44818
     *   2. EtherNet/IP session registration (CMD_REGISTER_SESSION)
     *   3. CIP service requests via SendRRData
     *   4. Session unregistration on disconnect
     */
    class EtherNetIPInterface : public PLCInterface
    {
    public:
        EtherNetIPInterface();
        ~EtherNetIPInterface() override;

        // ── Lifecycle ────────────────────────────────────────────────────

        bool configure(const std::unordered_map<std::string, std::string> &params) override;
        bool connect() override;
        bool disconnect() override;
        bool isConnected() const override;
        bool reconnect() override;

        // ── Tag I/O ─────────────────────────────────────────────────────

        IOResult readTag(const std::string &tag_name, PLCValue &value) override;
        IOResult writeTag(const std::string &tag_name, const PLCValue &value) override;
        IOResult readTagTyped(const std::string &tag_name, PLCValue &value, uint16_t expected_type);

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
        std::string getProtocolName() const override { return "EtherNet/IP (CIP)"; }

    private:
        // ── CIP Protocol Methods ────────────────────────────────────────

        /**
         * @brief Register an EtherNet/IP session with the PLC
         * @return true if session registration succeeds
         */
        bool registerSession();

        /**
         * @brief Unregister the current EtherNet/IP session
         */
        void unregisterSession();

        /**
         * @brief Build a CIP path from a symbolic tag name
         *
         * @details Converts tag names like "Program:Main.JointCmd[0]" into
         * the EPATH segment format required by CIP:
         *   - Symbolic segments for string components
         *   - Element segments for array indices
         *   - Member segments for structure fields
         *
         * @param tag_name The symbolic tag name
         * @return Byte vector containing the encoded CIP path
         */
        std::vector<uint8_t> buildTagPath(const std::string &tag_name);

        /**
         * @brief Build a CIP Read Tag service request
         */
        std::vector<uint8_t> buildReadRequest(const std::string &tag_name, uint16_t element_count = 1);

        /**
         * @brief Build a CIP Write Tag service request
         */
        std::vector<uint8_t> buildWriteRequest(const std::string &tag_name,
                                                uint16_t data_type,
                                                const std::vector<uint8_t> &data);

        /**
         * @brief Build a CIP Multiple Service Packet for batch operations
         *
         * @details Wraps multiple CIP service requests into a single
         * encapsulated message, reducing TCP round-trips. This is critical
         * for meeting real-time deadlines when the controller needs to
         * read/write many tags per control cycle.
         */
        std::vector<uint8_t> buildMultiServiceRequest(
            const std::vector<std::vector<uint8_t>> &service_requests);

        /**
         * @brief Wrap a CIP service request in an EtherNet/IP SendRRData message
         */
        std::vector<uint8_t> buildSendRRData(const std::vector<uint8_t> &cip_request);

        /**
         * @brief Send a message and receive the response
         * @return true if a valid response was received
         */
        bool sendReceive(const std::vector<uint8_t> &request, std::vector<uint8_t> &response);

        /**
         * @brief Parse a CIP read response and extract the value
         */
        IOResult parseReadResponse(const std::vector<uint8_t> &response, PLCValue &value);

        /**
         * @brief Parse a CIP write response and check for errors
         */
        IOResult parseWriteResponse(const std::vector<uint8_t> &response);

        /**
         * @brief Convert PLCValue to CIP type code and raw bytes
         */
        bool serializeValue(const PLCValue &value, uint16_t &type_code, std::vector<uint8_t> &data);

        /**
         * @brief Convert CIP raw bytes to PLCValue
         */
        bool deserializeValue(uint16_t type_code, const uint8_t *data, size_t length, PLCValue &value);

        /**
         * @brief Update round-trip timing statistics
         */
        void updateTimingStats(double round_trip_ms);

        // ── Connection State ────────────────────────────────────────────

        std::string ip_address_;
        uint16_t port_ = cip::DEFAULT_PORT;
        uint8_t slot_ = cip::DEFAULT_SLOT;
        uint32_t timeout_ms_ = cip::DEFAULT_TIMEOUT_MS;
        std::string routing_path_;

        int socket_fd_ = -1;
        uint32_t session_handle_ = 0;
        std::atomic<bool> connected_{false};

        mutable std::mutex io_mutex_;              ///< Protects socket I/O
        mutable std::mutex diagnostics_mutex_;     ///< Protects diagnostics counters

        // ── Diagnostics State ───────────────────────────────────────────

        ConnectionDiagnostics diagnostics_;

        // Tag cache for validation
        std::unordered_map<std::string, TagInfo> tag_cache_;
    };

}  // namespace plc_ros2_bridge

#endif  // PLC_ROS2_BRIDGE__ETHERNET_IP_INTERFACE_HPP_
