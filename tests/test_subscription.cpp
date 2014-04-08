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

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <boost/thread/mutex.hpp>
#include "boost/thread.hpp"

#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tf2_web_republisher/TFSubscriptionAction.h>

class TestSubscription
{
public:

  TestSubscription(const std::string& name) :
    ac_("tf2_web_republisher", true)
  {
    std::string sub_topic;
    nh_.param<std::string>("tf_topic", sub_topic, "/tf_web");
    sub_ = nh_.subscribe(sub_topic, 1, &TestSubscription::callbackTF, this);

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac_.waitForServer(); //will wait for infinite time

    ROS_INFO("Action server started, sending goal.");
    // send a goal to the action
    tf2_web_republisher::TFSubscriptionGoal goal;
    goal.source_frames.resize(1);
    goal.source_frames[0] = "base";
    goal.target_frame = "target";
    goal.angular_thres = 0.0f;
    goal.trans_thres = 0.0f;
    goal.rate = 5;

    ac_.sendGoal(goal, boost::bind(&TestSubscription::doneCb, this, _1, _2), boost::bind(&TestSubscription::activeCb, this), boost::bind(&TestSubscription::feedbackCb, this, _1));

    //wait for the action to return
    bool finished_before_timeout = ac_.waitForResult(ros::Duration(10.0));

    if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = ac_.getState();
      ROS_INFO("Action finished: %s", state.toString().c_str());
    }
    else
      ROS_INFO("Action did not finish before the time out.");
  }

  ~TestSubscription() {}

  // Called once when the goal completes
  void doneCb(const actionlib::SimpleClientGoalState& state,
              const tf2_web_republisher::TFSubscriptionResultConstPtr& result)
  {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    //ROS_INFO("Answer: %i", result->sequence.back());
    ros::shutdown();
  }

  // Called once when the goal becomes active
  void activeCb()
  {
    ROS_INFO("Goal just went active");
  }

  // Called every time feedback is received for the goal
  void feedbackCb(const tf2_web_republisher::TFSubscriptionFeedbackConstPtr& feedback)
  {
    ROS_INFO("Got Feedback of length %lu", feedback->transforms.size());
  }

  void callbackTF(const tf::tfMessageConstPtr &msg)
  {
    size_t i;
    size_t msg_size = msg->transforms.size();
    for (i = 0; i < msg_size; i++)
    {
      const geometry_msgs::TransformStamped& trans_msg = msg->transforms[i];

      const std::string& base_key = trans_msg.header.frame_id;
      const std::string& target_key = trans_msg.child_frame_id;

      ROS_INFO_STREAM("Received transform from "<<base_key<<" to "<<target_key);
    }
  }


protected:

  ros::NodeHandle nh_;
  ros::Subscriber sub_;

  actionlib::SimpleActionClient<tf2_web_republisher::TFSubscriptionAction> ac_;
};

int main(int argc, char **argv)
{
ros::init(argc, argv, "tf2_test_web_republisher");

TestSubscription tf2_test_republisher(ros::this_node::getName());
ros::spin();

return 0;
}
