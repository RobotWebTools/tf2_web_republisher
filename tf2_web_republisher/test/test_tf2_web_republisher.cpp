#include <gmock/gmock.h>

#include "tf2_web_republisher/tf2_web_republisher.hpp"

namespace
{
using tf2_web_republisher::TFRepublisher;
using GoalHandle = rclcpp_action::ClientGoalHandle<tf2_web_republisher_interfaces::action::TFSubscription>;

class TFRepublisherTest : public TFRepublisher
{
public:
  using TFRepublisher::clean_tf_frame;
  using TFRepublisher::execute;
  using TFRepublisher::handle_accepted;
  using TFRepublisher::handle_cancel;
  using TFRepublisher::handle_goal;
  using TFRepublisher::TFRepublisher;
  using TFRepublisher::thread_safe_lookup;

  std::shared_ptr<tf2_ros::Buffer> get_tf_buffer()
  {
    return tf_buffer_;
  }

  void goal_response_callback(const GoalHandle::SharedPtr& future)
  {
  }

  void
  feedback_callback(const GoalHandle::SharedPtr& gh,
                    const std::shared_ptr<const tf2_web_republisher_interfaces::action::TFSubscription::Feedback>& feedback)
  {
    feedback_received = true;
    feedback_msg = feedback;
  }

  void result_callback(const GoalHandle::WrappedResult& result)
  {
    result_received = true;
  }

  std::shared_ptr<const tf2_web_republisher_interfaces::action::TFSubscription::Feedback> feedback_msg;
  std::atomic<bool> feedback_received = false;
  std::atomic<bool> result_received = false;
};

TEST(TFWebRepublisher, TestActionCall)
{
  // create ROS thread
  auto tf2_web_republisher = std::make_shared<TFRepublisherTest>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(tf2_web_republisher);
  std::thread ros_thread([&executor]() { executor.spin(); });

  // create first TF message
  std_msgs::msg::Header header;
  header.frame_id = "world";
  header.stamp = rclcpp::Time(10, 0);
  geometry_msgs::msg::Transform tf;
  tf.translation.z = 10;
  auto tf_msg_1 =
      geometry_msgs::build<::geometry_msgs::msg::TransformStamped>().header(header).child_frame_id("frame_1").transform(
          tf);
  // create second TF message
  tf.translation.z = 5;

  // put messages in TF buffer
  auto tf_buffer = tf2_web_republisher->get_tf_buffer();
  tf_buffer->setTransform(tf_msg_1, "default_authority", false);
  auto tf_msg_2 =
      geometry_msgs::build<::geometry_msgs::msg::TransformStamped>().header(header).child_frame_id("frame_2").transform(
          tf);
  tf_buffer->setTransform(tf_msg_2, "default_authority", false);

  // create action client and test for server availability
  auto client_ptr = rclcpp_action::create_client<tf2_web_republisher_interfaces::action::TFSubscription>(
      tf2_web_republisher, "tf2_web_republisher");
  EXPECT_TRUE(client_ptr->wait_for_action_server()) << "Action server not available after waiting";

  // setup goal message
  auto goal_msg = tf2_web_republisher_interfaces::action::TFSubscription::Goal();
  goal_msg.source_frames = { "/frame_1" };
  goal_msg.target_frame = "/frame_2";
  goal_msg.rate = 10;

  // setup callbacks
  auto send_goal_options = rclcpp_action::Client<tf2_web_republisher_interfaces::action::TFSubscription>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::function<void(GoalHandle::SharedPtr)>([&tf2_web_republisher](const GoalHandle::SharedPtr& future) {
        tf2_web_republisher->goal_response_callback(future);
      });
  send_goal_options.feedback_callback = std::function<void(
      GoalHandle::SharedPtr,
      const std::shared_ptr<const tf2_web_republisher_interfaces::action::TFSubscription::Feedback> feedback)>(
      [&tf2_web_republisher](
          const GoalHandle::SharedPtr& gh,
          const std::shared_ptr<const tf2_web_republisher_interfaces::action::TFSubscription::Feedback>& feedback) {
        tf2_web_republisher->feedback_callback(gh, feedback);
      });
  send_goal_options.result_callback = std::function<void(const GoalHandle::WrappedResult&)>(
      [&tf2_web_republisher](const GoalHandle::WrappedResult& result) { tf2_web_republisher->result_callback(result); });
  client_ptr->async_send_goal(goal_msg, send_goal_options);

  // join threads
  while (!executor.is_spinning() || !tf2_web_republisher->feedback_received)
    ;
  client_ptr->async_cancel_all_goals();
  while (!tf2_web_republisher->result_received)
    ;
  executor.cancel();
  ros_thread.join();

  ASSERT_EQ(tf2_web_republisher->feedback_msg->transforms.size(), 1);
  EXPECT_EQ(tf2_web_republisher->feedback_msg->transforms[0].transform.translation.x, 0);
  EXPECT_EQ(tf2_web_republisher->feedback_msg->transforms[0].transform.translation.y, 0);
  EXPECT_EQ(tf2_web_republisher->feedback_msg->transforms[0].transform.translation.z, 5);
}

}  // namespace

int main(int argc, char** argv)
{
  testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
