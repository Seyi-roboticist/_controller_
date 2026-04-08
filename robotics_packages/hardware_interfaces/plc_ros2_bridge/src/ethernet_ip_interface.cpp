/**
 * @file ethernet_ip_interface.cpp
 * @author Seyi R. Afolayan
 * @brief EtherNet/IP (CIP) implementation for Allen-Bradley PLC communication
 *
 * @details Implements the PLCInterface for Allen-Bradley ControlLogix / CompactLogix
 * controllers using EtherNet/IP explicit messaging.
 *
 * @date 2025
 * @copyright MIT License
 */

#include "plc_ros2_bridge/ethernet_ip_interface.hpp"

#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <sys/socket.h>
#include <cstring>
#include <chrono>
#include <sstream>
#include <algorithm>
#include <stdexcept>

namespace plc_ros2_bridge
{
    // ─────────────────────────────────────────────────────────────────────────
    //  Construction / Destruction
    // ─────────────────────────────────────────────────────────────────────────

    EtherNetIPInterface::EtherNetIPInterface() = default;

    EtherNetIPInterface::~EtherNetIPInterface()
    {
        if (connected_)
        {
            disconnect();
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  Lifecycle
    // ─────────────────────────────────────────────────────────────────────────

    bool EtherNetIPInterface::configure(const std::unordered_map<std::string, std::string> &params)
    {
        // Required: IP address
        auto it = params.find("ip_address");
        if (it == params.end() || it->second.empty())
        {
            diagnostics_.last_error_message = "Missing required parameter: ip_address";
            return false;
        }
        ip_address_ = it->second;

        // Optional: port
        it = params.find("port");
        if (it != params.end())
        {
            port_ = static_cast<uint16_t>(std::stoi(it->second));
        }

        // Optional: slot
        it = params.find("slot");
        if (it != params.end())
        {
            slot_ = static_cast<uint8_t>(std::stoi(it->second));
        }

        // Optional: timeout
        it = params.find("timeout_ms");
        if (it != params.end())
        {
            timeout_ms_ = static_cast<uint32_t>(std::stoi(it->second));
        }

        // Optional: routing path (for multi-hop configurations)
        it = params.find("path");
        if (it != params.end())
        {
            routing_path_ = it->second;
        }

        diagnostics_.state = ConnectionState::DISCONNECTED;
        return true;
    }

    bool EtherNetIPInterface::connect()
    {
        std::lock_guard<std::mutex> lock(io_mutex_);

        if (connected_)
        {
            return true;
        }

        diagnostics_.state = ConnectionState::CONNECTING;

        // Create TCP socket
        socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (socket_fd_ == -1)
        {
            diagnostics_.state = ConnectionState::ERROR;
            diagnostics_.last_error_message = "Failed to create socket: " + std::string(strerror(errno));
            return false;
        }

        // Set socket timeout
        struct timeval tv;
        tv.tv_sec = timeout_ms_ / 1000;
        tv.tv_usec = (timeout_ms_ % 1000) * 1000;
        setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        setsockopt(socket_fd_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

        // Connect to PLC
        struct sockaddr_in server_addr{};
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(port_);

        if (inet_pton(AF_INET, ip_address_.c_str(), &server_addr.sin_addr) <= 0)
        {
            close(socket_fd_);
            socket_fd_ = -1;
            diagnostics_.state = ConnectionState::ERROR;
            diagnostics_.last_error_message = "Invalid IP address: " + ip_address_;
            return false;
        }

        if (::connect(socket_fd_, reinterpret_cast<struct sockaddr *>(&server_addr),
                       sizeof(server_addr)) == -1)
        {
            close(socket_fd_);
            socket_fd_ = -1;
            diagnostics_.state = ConnectionState::ERROR;
            diagnostics_.last_error_message = "Failed to connect to " + ip_address_ + ":" +
                                              std::to_string(port_) + " — " + strerror(errno);
            return false;
        }

        // Register EtherNet/IP session
        if (!registerSession())
        {
            close(socket_fd_);
            socket_fd_ = -1;
            diagnostics_.state = ConnectionState::ERROR;
            return false;
        }

        connected_ = true;
        diagnostics_.state = ConnectionState::CONNECTED;
        return true;
    }

    bool EtherNetIPInterface::disconnect()
    {
        std::lock_guard<std::mutex> lock(io_mutex_);

        if (!connected_)
        {
            return true;
        }

        // Unregister session
        if (session_handle_ != 0)
        {
            unregisterSession();
        }

        // Close socket
        if (socket_fd_ != -1)
        {
            close(socket_fd_);
            socket_fd_ = -1;
        }

        connected_ = false;
        session_handle_ = 0;
        diagnostics_.state = ConnectionState::DISCONNECTED;
        return true;
    }

    bool EtherNetIPInterface::isConnected() const
    {
        return connected_.load();
    }

    bool EtherNetIPInterface::reconnect()
    {
        diagnostics_.state = ConnectionState::RECONNECTING;

        // Close existing connection
        if (socket_fd_ != -1)
        {
            close(socket_fd_);
            socket_fd_ = -1;
        }
        connected_ = false;
        session_handle_ = 0;

        // Attempt reconnection
        bool result = connect();
        if (result)
        {
            std::lock_guard<std::mutex> lock(diagnostics_mutex_);
            diagnostics_.reconnect_count++;
        }
        return result;
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  Session Management
    // ─────────────────────────────────────────────────────────────────────────

    bool EtherNetIPInterface::registerSession()
    {
        // Build Register Session command
        // EtherNet/IP encapsulation header (24 bytes) + session data (4 bytes)
        std::vector<uint8_t> request(28, 0);

        // Command: Register Session (0x0065)
        request[0] = cip::CMD_REGISTER_SESSION & 0xFF;
        request[1] = (cip::CMD_REGISTER_SESSION >> 8) & 0xFF;

        // Length of command-specific data: 4 bytes
        request[2] = 4;
        request[3] = 0;

        // Session handle: 0 (requesting new session)
        // Already zero-initialized

        // Status: 0
        // Sender context: 0 (we could use this for tracking)
        // Options: 0

        // Command-specific data:
        // Protocol version: 1
        request[24] = 1;
        request[25] = 0;

        // Option flags: 0
        request[26] = 0;
        request[27] = 0;

        // Send and receive
        std::vector<uint8_t> response;
        if (!sendReceive(request, response))
        {
            diagnostics_.last_error_message = "Failed to send Register Session request";
            return false;
        }

        // Validate response
        if (response.size() < 28)
        {
            diagnostics_.last_error_message = "Register Session response too short";
            return false;
        }

        // Check status (bytes 8-11)
        uint32_t status = response[8] | (response[9] << 8) |
                          (response[10] << 16) | (response[11] << 24);
        if (status != 0)
        {
            diagnostics_.last_error_message = "Register Session failed with status: " +
                                              std::to_string(status);
            return false;
        }

        // Extract session handle (bytes 4-7)
        session_handle_ = response[4] | (response[5] << 8) |
                          (response[6] << 16) | (response[7] << 24);

        return true;
    }

    void EtherNetIPInterface::unregisterSession()
    {
        // Build Unregister Session command
        std::vector<uint8_t> request(24, 0);

        // Command: Unregister Session (0x0066)
        request[0] = cip::CMD_UNREGISTER_SESSION & 0xFF;
        request[1] = (cip::CMD_UNREGISTER_SESSION >> 8) & 0xFF;

        // Length: 0 (no command-specific data)

        // Session handle
        request[4] = session_handle_ & 0xFF;
        request[5] = (session_handle_ >> 8) & 0xFF;
        request[6] = (session_handle_ >> 16) & 0xFF;
        request[7] = (session_handle_ >> 24) & 0xFF;

        // Send without expecting response
        if (socket_fd_ != -1)
        {
            ::write(socket_fd_, request.data(), request.size());
        }

        session_handle_ = 0;
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  Tag I/O
    // ─────────────────────────────────────────────────────────────────────────

    IOResult EtherNetIPInterface::readTag(const std::string &tag_name, PLCValue &value)
    {
        std::lock_guard<std::mutex> lock(io_mutex_);

        if (!connected_)
        {
            return IOResult::CONNECTION_LOST;
        }

        auto start = std::chrono::steady_clock::now();

        // Build CIP Read Tag request
        auto cip_request = buildReadRequest(tag_name);
        auto encapsulated = buildSendRRData(cip_request);

        // Send and receive
        std::vector<uint8_t> response;
        if (!sendReceive(encapsulated, response))
        {
            std::lock_guard<std::mutex> dlock(diagnostics_mutex_);
            diagnostics_.failed_reads++;
            return IOResult::TIMEOUT;
        }

        // Parse the response
        IOResult result = parseReadResponse(response, value);

        auto end = std::chrono::steady_clock::now();
        double round_trip = std::chrono::duration<double, std::milli>(end - start).count();

        {
            std::lock_guard<std::mutex> dlock(diagnostics_mutex_);
            if (result == IOResult::SUCCESS)
            {
                diagnostics_.successful_reads++;
            }
            else
            {
                diagnostics_.failed_reads++;
            }
            updateTimingStats(round_trip);
        }

        return result;
    }

    IOResult EtherNetIPInterface::writeTag(const std::string &tag_name, const PLCValue &value)
    {
        std::lock_guard<std::mutex> lock(io_mutex_);

        if (!connected_)
        {
            return IOResult::CONNECTION_LOST;
        }

        auto start = std::chrono::steady_clock::now();

        // Serialize the value
        uint16_t type_code;
        std::vector<uint8_t> data;
        if (!serializeValue(value, type_code, data))
        {
            return IOResult::TYPE_MISMATCH;
        }

        // Build CIP Write Tag request
        auto cip_request = buildWriteRequest(tag_name, type_code, data);
        auto encapsulated = buildSendRRData(cip_request);

        // Send and receive
        std::vector<uint8_t> response;
        if (!sendReceive(encapsulated, response))
        {
            std::lock_guard<std::mutex> dlock(diagnostics_mutex_);
            diagnostics_.failed_writes++;
            return IOResult::TIMEOUT;
        }

        // Parse write response
        IOResult result = parseWriteResponse(response);

        auto end = std::chrono::steady_clock::now();
        double round_trip = std::chrono::duration<double, std::milli>(end - start).count();

        {
            std::lock_guard<std::mutex> dlock(diagnostics_mutex_);
            if (result == IOResult::SUCCESS)
            {
                diagnostics_.successful_writes++;
            }
            else
            {
                diagnostics_.failed_writes++;
            }
            updateTimingStats(round_trip);
        }

        return result;
    }

    IOResult EtherNetIPInterface::readTagTyped(const std::string &tag_name, PLCValue &value,
                                                uint16_t /*expected_type*/)
    {
        // For now, delegate to readTag. Type validation happens on the response.
        return readTag(tag_name, value);
    }

    std::unordered_map<std::string, IOResult> EtherNetIPInterface::readTags(
        const std::vector<std::string> &tag_names,
        std::unordered_map<std::string, PLCValue> &values)
    {
        std::unordered_map<std::string, IOResult> results;

        if (tag_names.empty())
        {
            return results;
        }

        // Build individual read requests
        std::vector<std::vector<uint8_t>> requests;
        requests.reserve(tag_names.size());
        for (const auto &tag : tag_names)
        {
            requests.push_back(buildReadRequest(tag));
        }

        // Wrap in Multiple Service Packet
        auto multi_request = buildMultiServiceRequest(requests);
        auto encapsulated = buildSendRRData(multi_request);

        // Send and receive
        std::vector<uint8_t> response;
        {
            std::lock_guard<std::mutex> lock(io_mutex_);

            if (!connected_)
            {
                for (const auto &tag : tag_names)
                {
                    results[tag] = IOResult::CONNECTION_LOST;
                }
                return results;
            }

            if (!sendReceive(encapsulated, response))
            {
                for (const auto &tag : tag_names)
                {
                    results[tag] = IOResult::TIMEOUT;
                }
                return results;
            }
        }

        // Parse individual responses from the multi-service response
        // For now, fall back to individual reads if multi-service parsing is complex
        for (const auto &tag : tag_names)
        {
            PLCValue val;
            IOResult res = readTag(tag, val);
            results[tag] = res;
            if (res == IOResult::SUCCESS)
            {
                values[tag] = val;
            }
        }

        return results;
    }

    std::unordered_map<std::string, IOResult> EtherNetIPInterface::writeTags(
        const std::unordered_map<std::string, PLCValue> &tag_values)
    {
        std::unordered_map<std::string, IOResult> results;

        // Write tags individually (batch optimization is protocol-specific)
        for (const auto &[tag, value] : tag_values)
        {
            results[tag] = writeTag(tag, value);
        }

        return results;
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  Tag Discovery
    // ─────────────────────────────────────────────────────────────────────────

    bool EtherNetIPInterface::getTagInfo(const std::string &tag_name, TagInfo &info)
    {
        // Check cache first
        auto it = tag_cache_.find(tag_name);
        if (it != tag_cache_.end())
        {
            info = it->second;
            return true;
        }

        // Attempt a test read to verify the tag exists
        PLCValue test_value;
        IOResult result = readTag(tag_name, test_value);

        if (result == IOResult::SUCCESS)
        {
            info.name = tag_name;
            info.is_writable = true;  // Assume writable unless we know otherwise
            info.is_safety_tag = false;

            // Determine type from the read value
            std::visit([&info](auto &&val)
            {
                using T = std::decay_t<decltype(val)>;
                if constexpr (std::is_same_v<T, bool>)
                    info.data_type = "BOOL";
                else if constexpr (std::is_same_v<T, int8_t>)
                    info.data_type = "SINT";
                else if constexpr (std::is_same_v<T, int16_t>)
                    info.data_type = "INT";
                else if constexpr (std::is_same_v<T, int32_t>)
                    info.data_type = "DINT";
                else if constexpr (std::is_same_v<T, float>)
                    info.data_type = "REAL";
                else if constexpr (std::is_same_v<T, double>)
                    info.data_type = "LREAL";
            }, test_value);

            tag_cache_[tag_name] = info;
            return true;
        }

        return false;
    }

    std::unordered_map<std::string, bool> EtherNetIPInterface::validateTags(
        const std::vector<std::string> &required_tags)
    {
        std::unordered_map<std::string, bool> results;
        for (const auto &tag : required_tags)
        {
            TagInfo info;
            results[tag] = getTagInfo(tag, info);
        }
        return results;
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  CIP Message Construction
    // ─────────────────────────────────────────────────────────────────────────

    std::vector<uint8_t> EtherNetIPInterface::buildTagPath(const std::string &tag_name)
    {
        std::vector<uint8_t> path;

        // CIP symbolic segment encoding:
        // Segment type 0x91 = symbolic segment
        // Followed by length byte, then ASCII tag name
        // Padded to even length

        // Split tag_name on '.' for structure members and '[]' for array indices
        std::string current = tag_name;

        // For now, encode the entire tag name as a single symbolic segment
        // Production implementations should parse structure members and array indices
        path.push_back(0x91);  // Symbolic segment type

        uint8_t name_length = static_cast<uint8_t>(current.length());
        path.push_back(name_length);

        for (char c : current)
        {
            path.push_back(static_cast<uint8_t>(c));
        }

        // Pad to even length
        if (current.length() % 2 != 0)
        {
            path.push_back(0x00);
        }

        return path;
    }

    std::vector<uint8_t> EtherNetIPInterface::buildReadRequest(const std::string &tag_name,
                                                                 uint16_t element_count)
    {
        auto tag_path = buildTagPath(tag_name);

        std::vector<uint8_t> request;

        // CIP service code: Read Tag (0x4C)
        request.push_back(cip::SVC_READ_TAG);

        // Path size in words (2-byte units)
        request.push_back(static_cast<uint8_t>(tag_path.size() / 2));

        // Append the tag path
        request.insert(request.end(), tag_path.begin(), tag_path.end());

        // Element count (number of elements to read)
        request.push_back(element_count & 0xFF);
        request.push_back((element_count >> 8) & 0xFF);

        return request;
    }

    std::vector<uint8_t> EtherNetIPInterface::buildWriteRequest(const std::string &tag_name,
                                                                  uint16_t data_type,
                                                                  const std::vector<uint8_t> &data)
    {
        auto tag_path = buildTagPath(tag_name);

        std::vector<uint8_t> request;

        // CIP service code: Write Tag (0x4D)
        request.push_back(cip::SVC_WRITE_TAG);

        // Path size in words
        request.push_back(static_cast<uint8_t>(tag_path.size() / 2));

        // Append the tag path
        request.insert(request.end(), tag_path.begin(), tag_path.end());

        // Data type
        request.push_back(data_type & 0xFF);
        request.push_back((data_type >> 8) & 0xFF);

        // Element count: 1
        request.push_back(1);
        request.push_back(0);

        // Data payload
        request.insert(request.end(), data.begin(), data.end());

        return request;
    }

    std::vector<uint8_t> EtherNetIPInterface::buildMultiServiceRequest(
        const std::vector<std::vector<uint8_t>> &service_requests)
    {
        std::vector<uint8_t> request;

        // CIP service code: Multiple Service Packet (0x0A)
        request.push_back(cip::SVC_MULTIPLE_SERVICE_PACKET);

        // Path: class 0x02 (Message Router), instance 1
        request.push_back(0x02);  // Path size: 2 words
        request.push_back(0x20);  // Class segment
        request.push_back(0x02);  // Class ID: Message Router
        request.push_back(0x24);  // Instance segment
        request.push_back(0x01);  // Instance ID: 1

        // Number of services
        uint16_t num_services = static_cast<uint16_t>(service_requests.size());
        request.push_back(num_services & 0xFF);
        request.push_back((num_services >> 8) & 0xFF);

        // Calculate offsets (relative to start of service data)
        uint16_t offset = static_cast<uint16_t>(2 + 2 * num_services);  // past count + offset table
        std::vector<uint16_t> offsets;

        for (const auto &svc : service_requests)
        {
            offsets.push_back(offset);
            offset += static_cast<uint16_t>(svc.size());
        }

        // Write offset table
        for (uint16_t off : offsets)
        {
            request.push_back(off & 0xFF);
            request.push_back((off >> 8) & 0xFF);
        }

        // Write service data
        for (const auto &svc : service_requests)
        {
            request.insert(request.end(), svc.begin(), svc.end());
        }

        return request;
    }

    std::vector<uint8_t> EtherNetIPInterface::buildSendRRData(const std::vector<uint8_t> &cip_request)
    {
        // Encapsulation header (24 bytes) + interface handle (4) + timeout (2)
        // + item count (2) + null address item (4) + data item header (4) + CIP data
        size_t total_data_length = 4 + 2 + 2 + 4 + 4 + cip_request.size();
        std::vector<uint8_t> message(24 + total_data_length, 0);

        // Command: SendRRData (0x006F)
        message[0] = cip::CMD_SEND_RR_DATA & 0xFF;
        message[1] = (cip::CMD_SEND_RR_DATA >> 8) & 0xFF;

        // Length
        uint16_t length = static_cast<uint16_t>(total_data_length);
        message[2] = length & 0xFF;
        message[3] = (length >> 8) & 0xFF;

        // Session handle
        message[4] = session_handle_ & 0xFF;
        message[5] = (session_handle_ >> 8) & 0xFF;
        message[6] = (session_handle_ >> 16) & 0xFF;
        message[7] = (session_handle_ >> 24) & 0xFF;

        // Command-specific data starts at offset 24
        size_t idx = 24;

        // Interface handle: 0
        idx += 4;

        // Timeout: 10 seconds
        message[idx++] = 10;
        message[idx++] = 0;

        // Item count: 2 (null address + unconnected data)
        message[idx++] = 2;
        message[idx++] = 0;

        // Item 1: Null Address Item (type 0x0000, length 0)
        message[idx++] = 0x00;
        message[idx++] = 0x00;
        message[idx++] = 0x00;
        message[idx++] = 0x00;

        // Item 2: Unconnected Data Item (type 0x00B2)
        message[idx++] = 0xB2;
        message[idx++] = 0x00;
        uint16_t data_len = static_cast<uint16_t>(cip_request.size());
        message[idx++] = data_len & 0xFF;
        message[idx++] = (data_len >> 8) & 0xFF;

        // CIP request payload
        std::copy(cip_request.begin(), cip_request.end(), message.begin() + idx);

        return message;
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  Network I/O
    // ─────────────────────────────────────────────────────────────────────────

    bool EtherNetIPInterface::sendReceive(const std::vector<uint8_t> &request,
                                            std::vector<uint8_t> &response)
    {
        if (socket_fd_ == -1)
        {
            return false;
        }

        // Send request
        ssize_t bytes_sent = ::write(socket_fd_, request.data(), request.size());
        if (bytes_sent != static_cast<ssize_t>(request.size()))
        {
            diagnostics_.last_error_message = "Failed to send: " + std::string(strerror(errno));
            connected_ = false;
            diagnostics_.state = ConnectionState::ERROR;
            return false;
        }

        // Read encapsulation header (24 bytes) to get response length
        std::vector<uint8_t> header(24);
        ssize_t bytes_read = ::read(socket_fd_, header.data(), 24);
        if (bytes_read != 24)
        {
            diagnostics_.last_error_message = "Failed to read response header";
            connected_ = false;
            diagnostics_.state = ConnectionState::ERROR;
            return false;
        }

        // Extract response data length from header (bytes 2-3)
        uint16_t data_length = header[2] | (header[3] << 8);

        // Read remaining data
        response = header;
        if (data_length > 0)
        {
            std::vector<uint8_t> data(data_length);
            size_t total_read = 0;

            while (total_read < data_length)
            {
                bytes_read = ::read(socket_fd_, data.data() + total_read,
                                     data_length - total_read);
                if (bytes_read <= 0)
                {
                    diagnostics_.last_error_message = "Failed to read response data";
                    connected_ = false;
                    diagnostics_.state = ConnectionState::ERROR;
                    return false;
                }
                total_read += bytes_read;
            }

            response.insert(response.end(), data.begin(), data.end());
        }

        return true;
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  Response Parsing
    // ─────────────────────────────────────────────────────────────────────────

    IOResult EtherNetIPInterface::parseReadResponse(const std::vector<uint8_t> &response,
                                                      PLCValue &value)
    {
        // Minimum response: 24 (encap header) + 16 (SendRRData overhead) + 4 (CIP response header)
        if (response.size() < 44)
        {
            diagnostics_.last_error_message = "Read response too short";
            return IOResult::PROTOCOL_ERROR;
        }

        // Navigate to CIP response data
        // After encapsulation header (24) + interface handle (4) + timeout (2)
        // + item count (2) + null address (4) + data item header (4)
        size_t cip_offset = 24 + 4 + 2 + 2 + 4 + 4;

        if (cip_offset >= response.size())
        {
            return IOResult::PROTOCOL_ERROR;
        }

        // CIP reply service code should be original | 0x80
        uint8_t reply_service = response[cip_offset];
        if ((reply_service & 0x80) == 0)
        {
            return IOResult::PROTOCOL_ERROR;
        }

        // Reserved byte
        // General status (0 = success)
        uint8_t status = response[cip_offset + 2];
        if (status != 0)
        {
            if (status == 0x05)
                return IOResult::TAG_NOT_FOUND;
            if (status == 0x13)
                return IOResult::TYPE_MISMATCH;
            return IOResult::PROTOCOL_ERROR;
        }

        // Additional status size
        uint8_t add_status_size = response[cip_offset + 3];
        size_t data_offset = cip_offset + 4 + (add_status_size * 2);

        if (data_offset + 2 >= response.size())
        {
            return IOResult::PROTOCOL_ERROR;
        }

        // Data type (2 bytes)
        uint16_t type_code = response[data_offset] | (response[data_offset + 1] << 8);
        data_offset += 2;

        // Deserialize value
        size_t remaining = response.size() - data_offset;
        if (!deserializeValue(type_code, &response[data_offset], remaining, value))
        {
            return IOResult::TYPE_MISMATCH;
        }

        return IOResult::SUCCESS;
    }

    IOResult EtherNetIPInterface::parseWriteResponse(const std::vector<uint8_t> &response)
    {
        if (response.size() < 44)
        {
            return IOResult::PROTOCOL_ERROR;
        }

        size_t cip_offset = 24 + 4 + 2 + 2 + 4 + 4;
        if (cip_offset + 3 >= response.size())
        {
            return IOResult::PROTOCOL_ERROR;
        }

        uint8_t status = response[cip_offset + 2];
        if (status == 0)
        {
            return IOResult::SUCCESS;
        }
        if (status == 0x05)
            return IOResult::TAG_NOT_FOUND;
        if (status == 0x13)
            return IOResult::TYPE_MISMATCH;
        if (status == 0x0E)
            return IOResult::PERMISSION_DENIED;

        return IOResult::PROTOCOL_ERROR;
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  Value Serialization
    // ─────────────────────────────────────────────────────────────────────────

    bool EtherNetIPInterface::serializeValue(const PLCValue &value, uint16_t &type_code,
                                               std::vector<uint8_t> &data)
    {
        return std::visit([&type_code, &data](auto &&val) -> bool
        {
            using T = std::decay_t<decltype(val)>;

            if constexpr (std::is_same_v<T, bool>)
            {
                type_code = cip::TYPE_BOOL;
                data.resize(1);
                data[0] = val ? 1 : 0;
            }
            else if constexpr (std::is_same_v<T, int8_t>)
            {
                type_code = cip::TYPE_SINT;
                data.resize(1);
                data[0] = static_cast<uint8_t>(val);
            }
            else if constexpr (std::is_same_v<T, int16_t>)
            {
                type_code = cip::TYPE_INT;
                data.resize(2);
                std::memcpy(data.data(), &val, 2);
            }
            else if constexpr (std::is_same_v<T, int32_t>)
            {
                type_code = cip::TYPE_DINT;
                data.resize(4);
                std::memcpy(data.data(), &val, 4);
            }
            else if constexpr (std::is_same_v<T, float>)
            {
                type_code = cip::TYPE_REAL;
                data.resize(4);
                std::memcpy(data.data(), &val, 4);
            }
            else if constexpr (std::is_same_v<T, double>)
            {
                type_code = cip::TYPE_LREAL;
                data.resize(8);
                std::memcpy(data.data(), &val, 8);
            }
            else
            {
                return false;
            }

            return true;
        }, value);
    }

    bool EtherNetIPInterface::deserializeValue(uint16_t type_code, const uint8_t *data,
                                                 size_t length, PLCValue &value)
    {
        size_t expected = cip::typeSize(type_code);
        if (expected == 0 || length < expected)
        {
            return false;
        }

        switch (type_code)
        {
        case cip::TYPE_BOOL:
            value = static_cast<bool>(data[0] != 0);
            break;
        case cip::TYPE_SINT:
            value = static_cast<int8_t>(data[0]);
            break;
        case cip::TYPE_INT:
        {
            int16_t v;
            std::memcpy(&v, data, 2);
            value = v;
            break;
        }
        case cip::TYPE_DINT:
        {
            int32_t v;
            std::memcpy(&v, data, 4);
            value = v;
            break;
        }
        case cip::TYPE_REAL:
        {
            float v;
            std::memcpy(&v, data, 4);
            value = v;
            break;
        }
        case cip::TYPE_LREAL:
        {
            double v;
            std::memcpy(&v, data, 8);
            value = v;
            break;
        }
        default:
            return false;
        }

        return true;
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  Diagnostics
    // ─────────────────────────────────────────────────────────────────────────

    ConnectionDiagnostics EtherNetIPInterface::getDiagnostics() const
    {
        std::lock_guard<std::mutex> lock(diagnostics_mutex_);
        return diagnostics_;
    }

    void EtherNetIPInterface::updateTimingStats(double round_trip_ms)
    {
        // Exponential moving average with alpha = 0.1
        constexpr double alpha = 0.1;
        if (diagnostics_.avg_round_trip_ms == 0.0)
        {
            diagnostics_.avg_round_trip_ms = round_trip_ms;
        }
        else
        {
            diagnostics_.avg_round_trip_ms =
                alpha * round_trip_ms + (1.0 - alpha) * diagnostics_.avg_round_trip_ms;
        }

        if (round_trip_ms > diagnostics_.max_round_trip_ms)
        {
            diagnostics_.max_round_trip_ms = round_trip_ms;
        }
    }

}  // namespace plc_ros2_bridge
