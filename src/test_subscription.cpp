/*********************************************************************
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

#include <actionlib/server/simple_action_server.h>
#include <tf2_web_republisher/TFSubscriptionAction.h>

class TestSubscription
{
public:

  TestSubscription(const std::string& name)
  {
    std::string sub_topic;
    nh_.param<std::string>("tf_topic", sub_topic, "/tf_web");
    sub_ = nh_.subscribe(sub_topic, 1, &TestSubscription::callbackTF, this);
  }

  ~TestSubscription() {}

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

  //actionlib::SimpleActionServer<tf2_web_republisher::TFSubscriptionAction> as_;
  tf2_web_republisher::TFSubscriptionActionFeedback action_feedback_;
  tf2_web_republisher::TFSubscriptionResult action_result_;

};

int main(int argc, char **argv)
{
ros::init(argc, argv, "tf2_test_web_republisher");

TestSubscription tf2_test_republisher(ros::this_node::getName());
ros::spin();

return 0;
}
