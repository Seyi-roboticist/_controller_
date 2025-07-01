#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <chrono>
#include <thread>
#include <cmath>

using namespace std::chrono_literals;

const std::string PARENT_FRAME = "base_link";
const std::string CHILD_FRAME = "target_sensor_frame";
const std::string END_EFFECTOR_FRAME = "tool0";
const double ERROR_THRESHOLD = 0.03;  // meters
const int CHECK_INTERVAL_MS = 2000;
const int MAX_WAIT_TIME_MS = 60000;

struct TransformTarget {
  double x, y, z, roll, pitch, yaw;
};

geometry_msgs::msg::TransformStamped create_static_transform(const TransformTarget &target) {
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = rclcpp::Clock().now();
  tf.header.frame_id = PARENT_FRAME;
  tf.child_frame_id = CHILD_FRAME;
  tf.transform.translation.x = target.x;
  tf.transform.translation.y = target.y;
  tf.transform.translation.z = target.z;

  tf2::Quaternion q;
  q.setRPY(target.roll, target.pitch, target.yaw);
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  tf.transform.rotation.w = q.w();

  return tf;
}

double euclidean_distance(const geometry_msgs::msg::Vector3 &a, const geometry_msgs::msg::Vector3 &b) {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("tf_monitor");

  // TF listener setup
  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);

  // Static transform broadcaster
  tf2_ros::StaticTransformBroadcaster static_broadcaster(node);

  std::vector<TransformTarget> targets = {
    {0.4, 0.2, 0.5, 0, 0, 0},
    {0.3, -0.3, 0.7, 0, 0, 0}
  };

  size_t current_index = 0;

  RCLCPP_INFO(node->get_logger(), "Starting transform monitor loop. Ctrl+C to exit.");

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  while (rclcpp::ok()) {
    auto &target = targets[current_index];
    auto tf_msg = create_static_transform(target);
    static_broadcaster.sendTransform(tf_msg);

    RCLCPP_INFO(node->get_logger(),
      "Published static transform to '%s': (%.3f, %.3f, %.3f)",
      CHILD_FRAME.c_str(), target.x, target.y, target.z);

    // Allow some time for movement to begin
    rclcpp::sleep_for(2s);

    bool target_reached = false;
    auto start_time = std::chrono::steady_clock::now();

    while (rclcpp::ok()) {
      try {
        auto transform = tf_buffer.lookupTransform(PARENT_FRAME, END_EFFECTOR_FRAME, tf2::TimePointZero);
        geometry_msgs::msg::Vector3 current_pos = transform.transform.translation;

        geometry_msgs::msg::Vector3 goal;
        goal.x = target.x;
        goal.y = target.y;
        goal.z = target.z;

        double distance = euclidean_distance(current_pos, goal);

        RCLCPP_INFO(node->get_logger(),
          "Current: (%.3f, %.3f, %.3f) | Target: (%.3f, %.3f, %.3f) | Distance: %.6f",
          current_pos.x, current_pos.y, current_pos.z,
          goal.x, goal.y, goal.z,
          distance);

        if (distance <= ERROR_THRESHOLD) {
          RCLCPP_INFO(node->get_logger(), "✅ Target reached within %.3f m threshold.", ERROR_THRESHOLD);
          target_reached = true;
          break;
        }
      } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(node->get_logger(), "TF lookup failed: %s", ex.what());
      }

      auto elapsed = std::chrono::steady_clock::now() - start_time;
      if (elapsed > std::chrono::milliseconds(MAX_WAIT_TIME_MS)) {
        RCLCPP_WARN(node->get_logger(), "Timeout reached. Moving to next target.");
        break;
      }

      rclcpp::sleep_for(std::chrono::milliseconds(CHECK_INTERVAL_MS));
    }

    if (target_reached) {
      rclcpp::sleep_for(3s);
    }

    current_index = (current_index + 1) % targets.size();
  }

  rclcpp::shutdown();
  return 0;
}
