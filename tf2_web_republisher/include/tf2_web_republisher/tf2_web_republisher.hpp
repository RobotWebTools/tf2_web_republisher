#pragma once

#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/transform_datatypes.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_web_republisher_interfaces/action/tf_subscription.hpp"
#include "tf2_web_republisher_interfaces/srv/republish_t_fs.hpp"

namespace tf2_web_republisher
{

class TFRepublisher : public rclcpp::Node
{
protected:
public:
  explicit TFRepublisher(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  ~TFRepublisher() override = default;

  rclcpp_action::CancelResponse handle_cancel(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<tf2_web_republisher_interfaces::action::TFSubscription>> /*gh*/);

  std::string clean_tf_frame(const std::string& frame_id) const;

  std::optional<geometry_msgs::msg::TransformStamped> thread_safe_lookup(const std::string& target_frame,
                                                                         const std::string& source_frame);

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID& /*uuid*/,
              const std::shared_ptr<const tf2_web_republisher_interfaces::action::TFSubscription::Goal>& /*goal*/);

  void handle_accepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<tf2_web_republisher_interfaces::action::TFSubscription>>&
          goal_handle);

  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<tf2_web_republisher_interfaces::action::TFSubscription>>&
                   goal_handle);

protected:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

private:
  rclcpp_action::Server<tf2_web_republisher_interfaces::action::TFSubscription>::SharedPtr action_server_;
  std::mutex tf_buffer_mutex_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{ nullptr };
};

}  // namespace tf2_web_republisher
