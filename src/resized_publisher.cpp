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

#include "resized_image_transport/resized_publisher.h"

#include <rclcpp/logging.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>

#include <cstdio> //for memcpy
#include <vector>
#include <opencv2/imgproc/imgproc.hpp>

namespace resized_image_transport
{

ResizedPublisher::ResizedPublisher()
: logger_(rclcpp::get_logger("ResizedPublisher")) {}

ResizedPublisher::~ResizedPublisher() {}

void ResizedPublisher::advertiseImpl(
  rclcpp::Node * node,
  const std::string & base_topic,
  uint32_t queue_size,
  rmw_qos_profile_t custom_qos)
{
  logger_ = node->get_logger();

  custom_qos.history = rmw_qos_profile_default.history;
  custom_qos.depth = queue_size + 4;

  typedef image_transport::SimplePublisherPlugin<resized_image_transport::msg::ResizedImage> Base;
  Base::advertiseImpl(node, base_topic, custom_qos);
}


void ResizedPublisher::publish(
  const sensor_msgs::msg::Image & message,
  const PublishFn & publish_fn) const
{
  // Convert ROS Image to OpenCV Image | sensor_msgs::msg::Image -> cv::Mat
  cv_bridge::CvImagePtr image_ptr;
  try {
    image_ptr = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(logger_, "cv_bridge exception: %s", e.what());
    return;
  }

  // Get OpenCV Image
  cv::Mat cv_image = image_ptr->image;

  // Rescale image
  double subsampling_factor = 2.0;
  int new_width = cv_image.cols / subsampling_factor + 0.5;
  int new_height = cv_image.rows / subsampling_factor + 0.5;
  cv::Mat cv_resized;
  cv::resize(cv_image, cv_resized, cv::Size(new_width, new_height));

  // Set up ResizedImage and publish
  resized_image_transport::msg::ResizedImage resized_image;
  resized_image.original_height = cv_image.rows;
  resized_image.original_width = cv_image.cols;
  // Convert OpenCV Image to ROS Image
  cv_bridge::CvImage image_bridge = cv_bridge::CvImage(
    message.header,
    sensor_msgs::image_encodings::BGR8,
    cv_resized);
  // from cv_bridge to sensor_msgs::Image
  image_bridge.toImageMsg(resized_image.image);

  // Publish
  publish_fn(resized_image);
}

} //namespace resized_image_transport
