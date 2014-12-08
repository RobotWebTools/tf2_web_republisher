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

#include <sstream>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <boost/thread/mutex.hpp>
#include "boost/thread.hpp"

#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_web_republisher/RepublishTFs.h>
#include <tf2_web_republisher/TFArray.h>

#include "tf_pair.h"

class TFRepublisher
{
protected:
  typedef tf2_web_republisher::RepublishTFs::Request Request;
  typedef tf2_web_republisher::RepublishTFs::Response Response;

  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;

  ros::ServiceServer tf_republish_service_;
  // struct that manages requests
  struct RequestInfo
  {
    ros::Publisher pub_;
    std::vector<TFPair> tf_subscriptions_;
    unsigned int request_ID_;
    ros::Timer timer_;
    ros::Duration unsub_timeout_;
    ros::Timer unsub_timer_;
  };

  std::list<boost::shared_ptr<RequestInfo> > active_requests_;
  boost::mutex requests_mutex_;

  // tf2 buffer and transformer
#if ROS_VERSION_MINOR < 10
  tf2::Buffer tf_buffer_;
  tf2::TransformListener tf_listener_;
#else
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
#endif
  boost::mutex tf_buffer_mutex_;

  unsigned int request_ID_count_;

public:

  TFRepublisher(const std::string& name) :
    nh_(),
    priv_nh_("~"),
    tf_listener_(tf_buffer_),
    request_ID_count_(0)
  {
    tf_republish_service_ = nh_.advertiseService("republish_tfs",
                                                 &TFRepublisher::requestCB,
                                                 this);
  }

  ~TFRepublisher() {}

  const std::string cleanTfFrame( const std::string frame_id ) const
  {
    if ( frame_id[0] == '/' )
    {
      return frame_id.substr(1);
    }
    return frame_id;
  }

  bool requestCB(Request& req, Response& res)
  {
    ROS_DEBUG("RepublishTF service request received");
    // generate request_info struct
    boost::shared_ptr<RequestInfo> request_info = boost::make_shared<RequestInfo>();

    std::stringstream topicname;
    topicname << "tf_repub_" << request_ID_count_;

    request_info->pub_ = priv_nh_.advertise<tf2_web_republisher::TFArray>(topicname.str(), 10, true);
    request_info->request_ID_ = request_ID_count_++;
    request_info->unsub_timeout_ = req.timeout;
    request_info->unsub_timer_ = nh_.createTimer(request_info->unsub_timeout_,
                                                 boost::bind(&TFRepublisher::unadvertiseCB, this, request_info, _1),
                                                 true); // only fire once
    request_info->timer_ = nh_.createTimer(ros::Duration(1.0 / req.rate),
                                           boost::bind(&TFRepublisher::processRequest, this, request_info, _1));

    std::size_t request_size_ = req.source_frames.size();
    request_info->tf_subscriptions_.resize(request_size_);

    for (std::size_t i=0; i<request_size_; ++i )
    {
      TFPair& tf_pair = request_info->tf_subscriptions_[i];

      std::string source_frame = cleanTfFrame(req.source_frames[i]);
      std::string target_frame = cleanTfFrame(req.target_frame);

      tf_pair.setSourceFrame(source_frame);
      tf_pair.setTargetFrame(target_frame);
      tf_pair.setAngularThres(req.angular_thres);
      tf_pair.setTransThres(req.trans_thres);
    }

    {
      boost::mutex::scoped_lock l(requests_mutex_);
      // add new request to list of active requests
      active_requests_.push_back(request_info);
    }
    res.topic_name = request_info->pub_.getTopic();
    ROS_INFO_STREAM("Publishing requested TFs on topic " << res.topic_name);

    return true;
  }

  void unadvertiseCB(boost::shared_ptr<RequestInfo> request_info, const ros::TimerEvent&)
  {
    ROS_INFO_STREAM("No subscribers on tf topic for request "
                     << request_info->request_ID_
                     << " for " << request_info->unsub_timeout_.toSec()
                     << " seconds. Unadvertising topic:"
                     << request_info->pub_.getTopic());
    request_info->pub_.shutdown();
    request_info->unsub_timer_.stop();
    request_info->timer_.stop();

    // search for RequestInfo struct and remove it from active_requests_ list
    for(std::list<boost::shared_ptr<RequestInfo> >::iterator it = active_requests_.begin(); it != active_requests_.end(); ++it)
    {
      RequestInfo& info = **it;
      if(info.pub_ == request_info->pub_)
      {
        active_requests_.erase(it);
        return;
      }
    }
  }

  void processRequest(boost::shared_ptr<RequestInfo> request_info, const ros::TimerEvent& )
  {
    if (request_info->pub_.getNumSubscribers() == 0)
    {
      request_info->unsub_timer_.start();
    }
    else
    {
      request_info->unsub_timer_.stop();
    }

    tf2_web_republisher::TFArray array_msg;
    {
      // iterate over tf_subscription map
      std::vector<TFPair>::iterator it ;
      std::vector<TFPair>::const_iterator end = request_info->tf_subscriptions_.end();

      for (it=request_info->tf_subscriptions_.begin(); it!=end; ++it)
      {
        geometry_msgs::TransformStamped transform;

        try
        {
          // protecting tf_buffer
          boost::mutex::scoped_lock lock (tf_buffer_mutex_);

          // lookup transformation for tf_pair
          transform = tf_buffer_.lookupTransform(it->getTargetFrame(),
                                                 it->getSourceFrame(),
                                                 ros::Time(0));

          // update tf_pair with transformtion
          it->updateTransform(transform);
        }
        catch (tf2::TransformException ex)
        {
          ROS_ERROR("%s", ex.what());
        }

        // check angular and translational thresholds
        if (it->updateNeeded())
        {
          transform.header.stamp = ros::Time::now();
          transform.header.frame_id = it->getTargetFrame();
          transform.child_frame_id = it->getSourceFrame();

          // notify tf_subscription that a network transmission has been triggered
          it->transmissionTriggered();

          // add transform to the array message
          array_msg.transforms.push_back(transform);
        }
      }
    }

    if (array_msg.transforms.size() > 0)
    {
      // publish TFs
      request_info->pub_.publish(array_msg);
      ROS_DEBUG("Request %d: TFs published:", request_info->request_ID_);
    }
    else
    {
      ROS_DEBUG("Request %d: No TF frame update needed:", request_info->request_ID_);
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf2_web_republisher");

  TFRepublisher tf2_web_republisher(ros::this_node::getName());
  ros::spin();

  return 0;
}
