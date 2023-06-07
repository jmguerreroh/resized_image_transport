/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2023, José Miguel Guerrero Hernández.
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
*********************************************************************/

#include "resized_image_transport/resized_subscriber.h"

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/logging.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

namespace resized_image_transport
{

ResizedSubscriber::ResizedSubscriber()
: logger_(rclcpp::get_logger("ResizedSubscriber")) {}

ResizedSubscriber::~ResizedSubscriber() {}

void ResizedSubscriber::subscribeImpl(
  rclcpp::Node * node,
  const std::string & base_topic,
  const Callback & callback,
  rmw_qos_profile_t custom_qos)
{
  this->subscribeImpl(node, base_topic, callback, custom_qos, rclcpp::SubscriptionOptions{});
}

void ResizedSubscriber::subscribeImpl(
  rclcpp::Node * node,
  const std::string & base_topic,
  const Callback & callback,
  rmw_qos_profile_t custom_qos,
  rclcpp::SubscriptionOptions options)
{
  logger_ = node->get_logger();
  typedef image_transport::SimpleSubscriberPlugin<resized_image_transport::msg::ResizedImage> Base;
  Base::subscribeImplWithOptions(node, base_topic, callback, custom_qos, options);
}


void ResizedSubscriber::internalCallback(
  const resized_image_transport::msg::ResizedImage::ConstSharedPtr & message,
  const Callback & callback)
{
  // Convert the image to OpenCV format
  cv_bridge::CvImagePtr img_rsz = cv_bridge::toCvCopy(message->image, message->image.encoding);

  // Resize the image to its original size
  cv::Mat img_restored;
  cv::resize(
    img_rsz->image, img_restored,
    cv::Size(message->original_width, message->original_height));

  sensor_msgs::msg::Image::SharedPtr send_image =
    cv_bridge::CvImage(message->image.header, message->image.encoding, img_restored).toImageMsg();

  callback(send_image);
}

} //namespace resized_image_transport
