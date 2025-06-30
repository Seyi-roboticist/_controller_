/**
 * @author Seyi R. Afolayan
 *
 * @brief The tf_look_up server is a ROS2 node I designed to bridge the gap between the ROS transformation (TF) system
 * and my hardware interface by using TCP/IP sockets.
 *
 * @cite https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp.html and Dr.Leonard's code
 */

#include "se3_sensor_driver/tf_lookup_server.hpp"

#include <rclcpp/serialization.hpp>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <sys/socket.h>
#include <fcntl.h> // For non-blocking socket
#include <mutex>   // For thread safety

namespace se3_sensor_driver
{
    ///////////////////////////////////////CONSTRUCTOR & DESTRUCTOR///////////////////////////////////////////////////////////
    TFLookupServer::TFLookupServer() : Node("tf_lookup_server"), sockfd_(-1), client_sockfd_(-1)
    {
        // Create separate callback groups
        timer_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        socket_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        // Declare parameters with default values
        this->declare_parameter("robot_parent_frame", "tool0");
        this->declare_parameter("robot_sensor_frame", "robot_sensor_frame");
        this->declare_parameter("target_parent_frame", "world");
        this->declare_parameter("target_sensor_frame", "target_sensor_frame");

        // Get the parameter values
        robot_parent_frame_ = this->get_parameter("robot_parent_frame").as_string();
        robot_sensor_frame_ = this->get_parameter("robot_sensor_frame").as_string();
        target_parent_frame_ = this->get_parameter("target_parent_frame").as_string();
        target_sensor_frame_ = this->get_parameter("target_sensor_frame").as_string();

        // Log the configured frames
        RCLCPP_INFO(this->get_logger(), "Configured with the following frames:");
        RCLCPP_INFO(this->get_logger(), "  Robot parent frame: %s", robot_parent_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Robot sensor frame: %s", robot_sensor_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Target parent frame: %s", target_parent_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Target sensor frame: %s", target_sensor_frame_.c_str());

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock()); // create a buffer that stores tf data
        // Here, I create a tf listener variable that subscribes to TF topics and populates the buffer with tf data
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

        setupServer();

        // Create a timer for sending TF data with its own callback group
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2),
            std::bind(&TFLookupServer::timerCallback, this),
            timer_callback_group_);

        // Create a timer for accepting socket connections
        socket_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2),
            std::bind(&TFLookupServer::socketAcceptCallback, this),
            socket_callback_group_);

        RCLCPP_INFO(this->get_logger(), "TF Lookup Server initialized and listening on port 12345");
    }

    TFLookupServer::~TFLookupServer()
    {
        if (client_sockfd_ != -1)
            close(client_sockfd_);

        if (sockfd_ != -1)
            close(sockfd_);
    }

    ////////////////////////////////////////METHOD 1 -> SETUPSERVER/////////////////////////////////////////////////
    void TFLookupServer::setupServer()
    {
        struct protoent *protoent = getprotobyname("tcp");         // get the protocol entry for TCP from the system's protocol database
        sockfd_ = socket(AF_INET, SOCK_STREAM, protoent->p_proto); // Create the TCP/IP socket

        // We perform sanity check to see if the socket creation failed.
        if (sockfd_ == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open socket.");
            return;
        }

        // I think this allows for reusing the socket address immediately after the process terminates
        int enable{1};
        if (setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable)) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to set socket option.");
            return;
        }

        // Make the socket non-blocking
        int flags = fcntl(sockfd_, F_GETFL, 0);
        fcntl(sockfd_, F_SETFL, flags | O_NONBLOCK);

        // We create and fill a socket address structure otherwise:
        struct sockaddr_in sockaddr;
        sockaddr.sin_family = AF_INET; // IPv4 address family
        sockaddr.sin_addr.s_addr = htonl(INADDR_ANY);
        sockaddr.sin_port = htons(12345);

        // Bind the socket to the specified address and port
        if (bind(sockfd_, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind socket: %s", strerror(errno));

            // Try alternate ports
            for (int port = 12346; port < 12356; port++)
            {
                sockaddr.sin_port = htons(port);
                if (bind(sockfd_, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) != -1)
                {
                    RCLCPP_INFO(this->get_logger(), "Successfully bound to port %d", port);
                    server_port_ = port;
                    break;
                }
            }

            // Check if we're still not bound
            if (bind(sockfd_, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) == -1)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to bind to any port");
                return;
            }
        }
        else
        {
            server_port_ = 12345;
        }

        // Listen for incoming connections
        if (listen(sockfd_, 5) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to listen");
            return;
        }

        // Otherwise, we proceed:
        RCLCPP_INFO(this->get_logger(), "Socket set up, waiting for client connections on port %d...", server_port_);
    }

    ////////////////////////////////////////METHOD 2 -> SOCKET ACCEPT CALLBACK/////////////////////////////////////////////////
    void TFLookupServer::socketAcceptCallback()
    {
        // Only try to accept if we don't have a client
        if (client_sockfd_ == -1)
        {
            struct sockaddr_in client_address;
            socklen_t client_len = sizeof(client_address);

            // Try to accept a connection (non-blocking)
            int new_client = accept(sockfd_, (struct sockaddr *)&client_address, &client_len);

            if (new_client != -1)
            {
                std::lock_guard<std::mutex> lock(client_mutex_);
                client_sockfd_ = new_client;
                RCLCPP_INFO(this->get_logger(), "Client connected from %s:%d",
                            inet_ntoa(client_address.sin_addr),
                            ntohs(client_address.sin_port));
            }
        }
    }

    ///////////////////////////////////////////METHOD3:CALLBACK//////////////////////////////////////////////////////////////
    void TFLookupServer::timerCallback()
    {
        std::lock_guard<std::mutex> lock(client_mutex_);

        // Only try to send data if we have a client
        if (client_sockfd_ == -1)
        {
            return;
        }

        // Pose messages
        geometry_msgs::msg::PoseStamped robot_pose;
        robot_pose.header.frame_id = this->robot_parent_frame_;
        robot_pose.header.stamp = this->now();

        try
        {
            // Try to get the transformation from the world to robot sensor
            auto robot_transform = tf_buffer_->lookupTransform(this->robot_parent_frame_, this->robot_sensor_frame_, tf2::TimePointZero);

            // Fill in pose from transform
            robot_pose.pose.position.x = robot_transform.transform.translation.x;
            robot_pose.pose.position.y = robot_transform.transform.translation.y;
            robot_pose.pose.position.z = robot_transform.transform.translation.z;
            robot_pose.pose.orientation = robot_transform.transform.rotation;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform for robot sensor: %s", ex.what());
            // Provide default values.. I am using this for testing
            robot_pose.pose.position.x = 0.0;
            robot_pose.pose.position.y = 0.0;
            robot_pose.pose.position.z = 0.0;
            robot_pose.pose.orientation.x = 0.0;
            robot_pose.pose.orientation.y = 0.0;
            robot_pose.pose.orientation.z = 0.0;
            robot_pose.pose.orientation.w = 1.0;
        }

        // Serialize the robot pose
        rclcpp::Serialization<geometry_msgs::msg::PoseStamped> serialization;
        rclcpp::SerializedMessage msg;
        serialization.serialize_message(&robot_pose, &msg);

        // First send the message size
        uint32_t message_size = msg.get_rcl_serialized_message().buffer_length;
        ssize_t size_bytes_written = write(client_sockfd_, &message_size, sizeof(message_size));

        if (size_bytes_written != sizeof(message_size))
        {
            if (errno == EPIPE || errno == ECONNRESET)
            {
                RCLCPP_INFO(this->get_logger(), "Client disconnected");
                close(client_sockfd_);
                client_sockfd_ = -1;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to write message size: %s", strerror(errno));
            }
            return;
        }

        // Then send the actual serialized message
        ssize_t bytes_written = write(client_sockfd_,
                                      msg.get_rcl_serialized_message().buffer,
                                      msg.get_rcl_serialized_message().buffer_length);

        if (bytes_written < 0 || static_cast<size_t>(bytes_written) < msg.get_rcl_serialized_message().buffer_length)
        {
            if (errno == EPIPE || errno == ECONNRESET)
            {
                RCLCPP_INFO(this->get_logger(), "Client disconnected");
                close(client_sockfd_);
                client_sockfd_ = -1;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to write complete message to socket: %s", strerror(errno));
            }
            return;
        }

        RCLCPP_DEBUG(this->get_logger(), "Sent robot sensor pose: [%f %f %f]",
                     robot_pose.pose.position.x,
                     robot_pose.pose.position.y,
                     robot_pose.pose.position.z);

        // Target pose message
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = target_parent_frame_;
        target_pose.header.stamp = this->now();

        // For target sensor, same pattern
        try
        {
            auto target_transform = tf_buffer_->lookupTransform(this->target_parent_frame_, this->target_sensor_frame_, tf2::TimePointZero);

            target_pose.pose.position.x = target_transform.transform.translation.x;
            target_pose.pose.position.y = target_transform.transform.translation.y;
            target_pose.pose.position.z = target_transform.transform.translation.z;
            target_pose.pose.orientation = target_transform.transform.rotation;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform for target sensor: %s", ex.what());
            // Provide default values with target offset
            target_pose.pose.position.x = 1.0; // 1 meter away in x
            target_pose.pose.position.y = 0.0;
            target_pose.pose.position.z = 0.0;
            target_pose.pose.orientation.x = 0.0;
            target_pose.pose.orientation.y = 0.0;
            target_pose.pose.orientation.z = 0.0;
            target_pose.pose.orientation.w = 1.0;
        }

        // Serialize the target pose
        serialization.serialize_message(&target_pose, &msg);

        // First send the message size
        message_size = msg.get_rcl_serialized_message().buffer_length;
        size_bytes_written = write(client_sockfd_, &message_size, sizeof(message_size));

        if (size_bytes_written != sizeof(message_size))
        {
            if (errno == EPIPE || errno == ECONNRESET)
            {
                RCLCPP_INFO(this->get_logger(), "Client disconnected");
                close(client_sockfd_);
                client_sockfd_ = -1;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to write message size for target sensor: %s", strerror(errno));
            }
            return;
        }

        // Then send the actual serialized message
        bytes_written = write(client_sockfd_,
                              msg.get_rcl_serialized_message().buffer,
                              msg.get_rcl_serialized_message().buffer_length);

        if (bytes_written < 0 || static_cast<size_t>(bytes_written) < msg.get_rcl_serialized_message().buffer_length)
        {
            if (errno == EPIPE || errno == ECONNRESET)
            {
                RCLCPP_INFO(this->get_logger(), "Client disconnected");
                close(client_sockfd_);
                client_sockfd_ = -1;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to write complete message for target sensor: %s", strerror(errno));
            }
            return;
        }

        RCLCPP_DEBUG(this->get_logger(), "Sent target sensor pose: [%f, %f, %f]",
                     target_pose.pose.position.x,
                     target_pose.pose.position.y,
                     target_pose.pose.position.z);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<se3_sensor_driver::TFLookupServer>();

    // Use a multithreaded executor to handle multiple callback groups
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}