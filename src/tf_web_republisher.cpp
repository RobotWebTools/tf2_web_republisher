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

#include "tf_pair.h"

class TFRepublisher
{
public:

  TFRepublisher(const std::string& name) :
      as_(nh_, name, boost::bind(&TFRepublisher::TFsubscribe, this, _1), false),
      tf_listener_(tf_buffer_)
  {
    as_.start();

    std::string pub_topic;
    nh_.param<std::string>("publish_topic", pub_topic, "/tf_web");
    pub_ = nh_.advertise<tf::tfMessage>(pub_topic, 1);

    double rate;
    nh_.param<double>("rate", rate, 10.0);
    republish_timer_ = nh_.createTimer(ros::Duration(1.0 / rate), boost::bind(&TFRepublisher::republish, this));
  }

  ~TFRepublisher() {}


  void TFsubscribe(const tf2_web_republisher::TFSubscriptionGoalConstPtr &goal)
  {

    TFPairPtr tf_pair = TFPairPtr(new TFPair());

    tf_pair->setSourceFrame(goal->source_frame);
    tf_pair->setTargetFrame(goal->target_frame);
    tf_pair->setAngularThres(goal->angular_thres);
    tf_pair->setTransThres(goal->trans_thres);

    {
      boost::mutex::scoped_lock lock (mutex_);
      tf_subscriptions_[tf_pair->getID()]=tf_pair;
    }

    ROS_DEBUG_STREAM("Action received: "<<tf_pair->getID());

    as_.setSucceeded(action_result_);

  }

  void republish()
  {
    while (nh_.ok())
    {
      {
        boost::mutex::scoped_lock lock (mutex_);

        tf::tfMessage msg;

        // iterate over tf_subscription map
        std::map<std::string, TFPairPtr>::iterator it;
        std::map<std::string, TFPairPtr>::const_iterator end = tf_subscriptions_.end();

        for (it=tf_subscriptions_.begin(); it!=end; ++it)
        {
          TFPairPtr tf_pair = it->second;

          geometry_msgs::TransformStampedPtr transform;
          try
          {
            // lookup transformation for tf_pair
            *transform = tf_buffer_.lookupTransform(tf_pair->getSourceFrame(),
                                                    tf_pair->getTargetFrame(),
                                                    ros::Time(0));
            // update tf_pair with transformtion
            tf_pair->setTransform(transform);
          }
          catch (tf2::TransformException ex)
          {
            ROS_ERROR("%s", ex.what());
          }

          // check angular and translational thresholds
          if (tf_pair->updateNeeded())
          {
            // add transformation to tf output message
            const geometry_msgs::TransformStampedConstPtr stamped_msg = transform;
            if (stamped_msg)
            {
              msg.transforms.push_back(*stamped_msg);
              // setting time & frame information
              msg.transforms.back().header.stamp = ros::Time::now();
              msg.transforms.back().header.frame_id = tf_pair->getSourceFrame();
              msg.transforms.back().child_frame_id = tf_pair->getTargetFrame();
            }

          }
        }
        if (msg.transforms.size()>0)
          pub_.publish(msg);
      }
    }
  }

protected:

  ros::NodeHandle nh_;
  ros::Publisher pub_;

  actionlib::SimpleActionServer<tf2_web_republisher::TFSubscriptionAction> as_;
  tf2_web_republisher::TFSubscriptionActionFeedback action_feedback_;
  tf2_web_republisher::TFSubscriptionResult action_result_;

  boost::mutex mutex_;
  ros::Timer republish_timer_;

  tf2::Buffer tf_buffer_;
  tf2::TransformListener tf_listener_;

  std::map<std::string, TFPairPtr> tf_subscriptions_;

  std::string base_frame_;

};

int main(int argc, char **argv)
{
ros::init(argc, argv, "tf2_web_republisher");

TFRepublisher tf2_web_republisher(ros::this_node::getName());
ros::spin();

return 0;
}
