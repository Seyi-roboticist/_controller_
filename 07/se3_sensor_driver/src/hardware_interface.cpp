/**
 * @author Seyi R. Afolayan
 * @date 31-Mar-2025
 *
 * @cite ros2_control_demos (via humble branch)
 */

#include <se3_sensor_driver/hardware_interface.hpp>

#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <random>
#include <ctime>  // Time (nullptr)
#include <thread> // For sleep in reconnection attempts

// Includes for socket communication
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <sys/socket.h>
#include <rclcpp/serialization.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <pluginlib/class_list_macros.hpp> // for exporting the driver

namespace se3_sensor_driver
{
    ////////////////////////////////////////////////////CALLBACKRETURN ON INITIALIZATION//////////////////////////////////////////////
    hardware_interface::CallbackReturn SE3SensorHardware::on_init(const hardware_interface::HardwareInfo &hardware_info)
    {
        if (hardware_interface::SensorInterface::on_init(hardware_info) != hardware_interface::CallbackReturn::SUCCESS)
            return hardware_interface::CallbackReturn::ERROR;

        // OTHERWISE:
        // Create a logger for the sensor
        this->logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger(
            "controller_manager.resource_manager.hardware_component.sensor.se3_sensor_driver"));

        this->clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());

        // Get parameters from hardware_info --> Checks the se3_sensor.xacro for this params and get their information.
        this->ip_address_ = info_.hardware_parameters.at("ip_address");

        /* This is where I initialize the socket connection variables */
        this->sockfd_ = -1;
        this->connected_ = false;
        this->reconnect_attempts_ = 0;
        this->max_reconnect_attempts_ = 5;
        this->reconnect_delay_ms_ = 1000; // delay for 1s (I might tweak this later)

        RCLCPP_INFO(this->get_logger(), "Initializing SE3 Sensor with IP: %s", ip_address_.c_str());

        // Extract frame_ids for each sensor (got idea from the API) ---> The `sensor.parameter` is a field that holds a map of params from URDF
        for (const auto &sensor : info_.sensors)
        {
            if (sensor.parameters.count("frame_id") > 0)
            { /* count() return 1 or 0 if the key exist or not respectively */
                sensor_frame_ids_[sensor.name] = sensor.parameters.at("frame_id");
                RCLCPP_INFO(this->get_logger(), "Sensor [%s] with frame_id: %s", sensor.name.c_str(), sensor_frame_ids_[sensor.name].c_str());
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "No frame_id parameter provided for sensor [%s], using default", sensor.name.c_str());
                sensor_frame_ids_[sensor.name] = sensor.name + "_frame";
            }
        }

        // Initialize sensor states vector for 3D pos and rot [x y z qx qy qz qw]
        this->hw_sensor_states_.resize(info_.sensors.size() * 7, std::numeric_limits<double>::quiet_NaN());

        RCLCPP_INFO(this->get_logger(), "SE3 Sensor successfully initialized.");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    ////////////////////////////////////////////////////EXPORT STATE INTERFACE ////////////////////////////////////////////////////////
    std::vector<hardware_interface::StateInterface> SE3SensorHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        // Sanity Check if we have any sensors
        if (info_.sensors.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "No sensors found in the hardware info");
            return state_interfaces;
        }

        // Print all sensor names and parameters for debugging
        for (size_t i = 0; i < info_.sensors.size(); ++i)
        {
            const auto &sensor = info_.sensors[i];
            RCLCPP_INFO(this->get_logger(), "Sensor %zu name: %s", i, sensor.name.c_str());
            for (const auto &param : sensor.parameters)
            {
                RCLCPP_INFO(this->get_logger(), "  Parameter: %s = %s",
                            param.first.c_str(), param.second.c_str());
            }
        }

        // Use a loop for looping through all sensors in the hardware info
        for (size_t i = 0; i < info_.sensors.size(); ++i)
        {
            const auto &sensor = info_.sensors[i];
            size_t base_idx = i * 7; // Each sensor uses 7 values

            // Log the exact sensor name we're exporting
            RCLCPP_INFO(this->get_logger(), "Exporting interfaces for sensor: %s", sensor.name.c_str());

            // Position interfaces [x y z]
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                sensor.name, "position.x", &hw_sensor_states_[base_idx + 0]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                sensor.name, "position.y", &hw_sensor_states_[base_idx + 1]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                sensor.name, "position.z", &hw_sensor_states_[base_idx + 2]));

            // Orientation interfaces [qx qy qz qw]
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                sensor.name, "orientation.x", &hw_sensor_states_[base_idx + 3]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                sensor.name, "orientation.y", &hw_sensor_states_[base_idx + 4]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                sensor.name, "orientation.z", &hw_sensor_states_[base_idx + 5]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                sensor.name, "orientation.w", &hw_sensor_states_[base_idx + 6]));
        }

        return state_interfaces;
    }

    ////////////////////////////////////////////////////////////CALLBACKRETURN ON ACTIVATE/////////////////////////////////////////////////
    hardware_interface::CallbackReturn SE3SensorHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(this->get_logger(), "Activating SE3 sensor system...");

        // Initialize all sensor values
        hw_sensor_states_.resize(info_.sensors.size() * 7, 0.0);

        // Similar to assignment 6, I will set the orientation to identity here.. ie qw=1.
        for (size_t i = 0; i < info_.sensors.size(); ++i)
        {
            hw_sensor_states_[i * 7 + 6] = 1.0;
        }

        /* Try to connect to server during activation */
        this->reconnect_attempts_ = 0;
        if (!connectToServer())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to TF lookup server during activation after %d attempts", max_reconnect_attempts_);
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(this->get_logger(), "SE3 sensor successfully activated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    ///////////////////////////////////////////////////////////CALLBACKRETURN ON DEACTIVATE///////////////////////////////////////////////
    hardware_interface::CallbackReturn SE3SensorHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(this->get_logger(), "Deactivating SE3 sensor  system...");

        /* Close socket if connected */
        if (connected_)
        {
            close(sockfd_);
            sockfd_ = -1;
            connected_ = false;
            RCLCPP_INFO(this->get_logger(), "Close communication to TF lookup server");
        }

        RCLCPP_INFO(this->get_logger(), "SE3 sensor successfully deactivated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    ///////////////////////////////////////////////////////////CONNECT TO SERVER //////////////////////////////////////////////////
    bool SE3SensorHardware::connectToServer()
    {
        if (connected_)
            return true;

        // If we have exceed the maximum number of reconnection attempts, give up
        if (reconnect_attempts_ >= max_reconnect_attempts_)
        {
            RCLCPP_ERROR(this->get_logger(), "Maximum reconnection attempts (%d) reached. Giving up.", max_reconnect_attempts_);
            return false;
        }

        reconnect_attempts_++;
        RCLCPP_INFO(this->get_logger(), "Attempting to connect to server (attempt %d of %d)...", reconnect_attempts_, max_reconnect_attempts_);

        struct protoent *protoent = getprotobyname("tcp");
        sockfd_ = socket(AF_INET, SOCK_STREAM, protoent->p_proto);

        if (sockfd_ == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
            std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_delay_ms_));
            return connectToServer();
        }

        struct hostent *hostent = gethostbyname("127.0.0.1"); // using localhost on pc or stuff
        if (!hostent)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to resolve hostname");
            close(sockfd_);
            sockfd_ = -1;
            std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_delay_ms_));
            return connectToServer(); // Retry
        }

        struct sockaddr_in sockaddr;
        sockaddr.sin_addr.s_addr = inet_addr(inet_ntoa(*(struct in_addr *)*(hostent->h_addr_list)));
        sockaddr.sin_family = AF_INET;
        sockaddr.sin_port = htons(12345);

        if (connect(sockfd_, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to server");
            close(sockfd_);
            sockfd_ = -1;
            std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_delay_ms_));
            return connectToServer(); // Retry
        }

        connected_ = true;
        reconnect_attempts_ = 0; // reset the counter on successful connection
        RCLCPP_INFO(this->get_logger(), "Connected to TF lookup server");
        return true;
    }

    ///////////////////////////////////////////////////////////READ STATE INTERFACES///////////////////////////////////////////////
    hardware_interface::return_type SE3SensorHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        // Try to connect to the server if not connected
        if (!connected_ && !connectToServer())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to TF lookup server. Connection is required.");
            return hardware_interface::return_type::ERROR;
        }

        // Connected to server, read data for each sensor
        for (size_t sensor_idx = 0; sensor_idx < info_.sensors.size(); ++sensor_idx)
        {
            // Base index for this sensor's data in hw_sensor_states_
            size_t base_idx = sensor_idx * 7;
            const auto &sensor = info_.sensors[sensor_idx];
            std::string frame_id = sensor_frame_ids_[sensor.name];

            uint32_t message_size{0}; // I used a msg of 128 initially but it didnt work
            // Read data from socket --> I am using ::read to prevent recursive call and use the global namespace
            ssize_t size_bytes_read = ::read(sockfd_, &message_size, sizeof(message_size));

            if (size_bytes_read != sizeof(message_size))
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to read message size for sensor [%s]. Read %zd bytes, expected %zu bytes",
                             sensor.name.c_str(), size_bytes_read, sizeof(message_size));

                close(sockfd_);
                sockfd_ = -1;
                connected_ = false;
                return hardware_interface::return_type::ERROR;
            }

            // I allocate a buffer of the correct size
            rclcpp::SerializedMessage msg(message_size); // I will need to match this in the server.cpp (the node)

            // Read the full message with multiple reads if necessary
            size_t total_read{0};
            while (total_read < message_size)
            {
                ssize_t bytes_read = ::read(sockfd_,
                                            msg.get_rcl_serialized_message().buffer + total_read,
                                            message_size - total_read);

                if (bytes_read <= 0)
                {
                    RCLCPP_ERROR(this->get_logger(), "Error reading message payload for sensor [%s] at byte offset %zu",
                                 sensor.name.c_str(), total_read);
                    close(sockfd_);
                    sockfd_ = -1;
                    connected_ = false;
                    return hardware_interface::return_type::ERROR;
                }

                total_read += bytes_read;
            }

            // Set the correct buffer length
            msg.get_rcl_serialized_message().buffer_length = message_size;

            try
            {
                // Deserialize the message
                rclcpp::Serialization<geometry_msgs::msg::PoseStamped> deserialization;
                geometry_msgs::msg::PoseStamped pose;
                deserialization.deserialize_message(&msg, &pose);

                // Update sensor state with received pose
                hw_sensor_states_[base_idx + 0] = pose.pose.position.x;
                hw_sensor_states_[base_idx + 1] = pose.pose.position.y;
                hw_sensor_states_[base_idx + 2] = pose.pose.position.z;
                hw_sensor_states_[base_idx + 3] = pose.pose.orientation.x;
                hw_sensor_states_[base_idx + 4] = pose.pose.orientation.y;
                hw_sensor_states_[base_idx + 5] = pose.pose.orientation.z;
                hw_sensor_states_[base_idx + 6] = pose.pose.orientation.w;

                // Log every 100th reading
                static uint8_t counter = 0;
                if (++counter % 100 == 0)
                {
                    std::stringstream ss;
                    ss << "SE3 Sensor [" << sensor.name << "] with frame [" << frame_id << "] readings: ";
                    ss << "Position [" << std::fixed << std::setprecision(3)
                       << hw_sensor_states_[base_idx + 0] << ", "
                       << hw_sensor_states_[base_idx + 1] << ", "
                       << hw_sensor_states_[base_idx + 2] << "] ";
                    ss << "Orientation ["
                       << hw_sensor_states_[base_idx + 3] << ", "
                       << hw_sensor_states_[base_idx + 4] << ", "
                       << hw_sensor_states_[base_idx + 5] << ", "
                       << hw_sensor_states_[base_idx + 6] << "]";

                    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to deserialize message for sensor [%s]: %s",
                             sensor.name.c_str(), e.what());

                // For debugging, I am just printing the raw buffer content
                std::stringstream hex_dump;
                hex_dump << "Raw message content (first 32 bytes or less): ";
                for (size_t i = 0; i < std::min(message_size, (uint32_t)32); ++i)
                {
                    hex_dump << std::hex << std::setw(2) << std::setfill('0')
                             << static_cast<int>(msg.get_rcl_serialized_message().buffer[i]) << " ";
                }
                RCLCPP_ERROR(this->get_logger(), "%s", hex_dump.str().c_str());

                return hardware_interface::return_type::ERROR;
            }
        }
        return hardware_interface::return_type::OK;
    }
}

PLUGINLIB_EXPORT_CLASS(
    se3_sensor_driver::SE3SensorHardware, hardware_interface::SensorInterface)