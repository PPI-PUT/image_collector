// Copyright 2023 Amadeusz Szymko
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "image_collector/image_collector_node.hpp"

namespace image_collector
{

ImageCollectorNode::ImageCollectorNode(const rclcpp::NodeOptions & options)
:  Node("image_collector", options)
{
  image_collector_ = std::make_unique<image_collector::ImageCollector>();
  auto save_format = this->declare_parameter<std::string>("save_format", "png");
  auto save_dir = this->declare_parameter<std::string>("save_dir", "");
  image_collector_->setParameters(save_format, save_dir);

  rgb_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::Image>>(
    this, "image_rgb");
  depth_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::Image>>(
    this, "image_depth");
  imgs_sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(10), *rgb_sub_, *depth_sub_);
  imgs_sync_->registerCallback(
    std::bind(
      &ImageCollectorNode::imagesCallback, this, std::placeholders::_1,
      std::placeholders::_2));
  save_service_ = this->create_service<std_srvs::srv::Trigger>(
    "~/save_images", std::bind(
      &ImageCollectorNode::saveService, this, std::placeholders::_1,
      std::placeholders::_2));
}

void ImageCollectorNode::imagesCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & rgb_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg)
{
  rgb_msg_ = rgb_msg;
  depth_msg_ = depth_msg;
}

void ImageCollectorNode::saveService(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (rgb_msg_ == nullptr || depth_msg_ == nullptr) {
    response->success = false;
    response->message = "No images received.";
    return;
  }
  auto img_rgb = cv_bridge::toCvShare(rgb_msg_, "bgr8")->image;
  auto img_depth = cv_bridge::toCvShare(depth_msg_, "32FC1")->image;
  image_collector_->saveImages(img_rgb, img_depth);
  response->success = true;
  response->message = "Images saved.";
}


}  // namespace image_collector

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(image_collector::ImageCollectorNode)
