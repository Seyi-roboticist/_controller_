/**
 * @author Seyi R. Afolayan 
 */

 #if !defined(TF_LOOKUP_SERVER_HPP_)
 #define TF_LOOKUP_SERVER_HPP_
 
 #include <rclcpp/rclcpp.hpp>
 #include <tf2_ros/buffer.h>
 #include <tf2_ros/transform_listener.h>
 #include <geometry_msgs/msg/pose_stamped.hpp>
 #include <mutex>
 
 namespace se3_sensor_driver {
     class TFLookupServer : public rclcpp::Node {
     public: 
         // Constructors and Destructors 
         TFLookupServer();
         ~TFLookupServer();// figure out why default didn't work (Is it because we need a custom destructor to clean up socket resources?)
     
     private:
         // Private methods 
         void setupServer();
         void socketAcceptCallback();
         void timerCallback(); 
 
         // Member variables 
         std::unique_ptr<tf2_ros::Buffer> tf_buffer_; 
         std::unique_ptr<tf2_ros::TransformListener> tf_listener_; 
         
         // Callback groups for multithreading
         rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
         rclcpp::CallbackGroup::SharedPtr socket_callback_group_;
         
         // Timers
         rclcpp::TimerBase::SharedPtr timer_;
         rclcpp::TimerBase::SharedPtr socket_timer_;
         
         // Mutex for thread safety
         std::mutex client_mutex_;
         
         // Frame IDs from parameters
         std::string robot_parent_frame_, robot_sensor_frame_, target_sensor_frame_, target_parent_frame_;
         
         // Socket variables
         int sockfd_, client_sockfd_;
         int server_port_ = 12345;
     };
 }
 
 #endif // TF_LOOKUP_SERVER_HPP_