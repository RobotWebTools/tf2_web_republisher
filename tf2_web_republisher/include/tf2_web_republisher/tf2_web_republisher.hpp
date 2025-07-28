/*********************************************************************
 *
 *  Copyright (c) 2014, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.

 *  Author: Julius Kammerl (jkammerl@willowgarage.com)
 *
 */

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

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& /*uuid*/,
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
