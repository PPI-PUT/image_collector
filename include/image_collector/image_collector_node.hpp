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

#ifndef IMAGE_COLLECTOR__IMAGE_COLLECTOR_NODE_HPP_
#define IMAGE_COLLECTOR__IMAGE_COLLECTOR_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "cv_bridge/cv_bridge.h"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "image_collector/image_collector.hpp"

namespace image_collector
{
using ImageCollectorPtr = std::unique_ptr<image_collector::ImageCollector>;
typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncPolicy;

class IMAGE_COLLECTOR_PUBLIC ImageCollectorNode : public rclcpp::Node
{
public:
  explicit ImageCollectorNode(const rclcpp::NodeOptions & options);

private:
  ImageCollectorPtr image_collector_{nullptr};
  sensor_msgs::msg::Image::ConstSharedPtr rgb_msg_{nullptr};
  sensor_msgs::msg::Image::ConstSharedPtr depth_msg_{nullptr};
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> rgb_sub_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> imgs_sync_;
  void imagesCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & rgb_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg);
  std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> save_service_;
  void saveService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response);
};
}  // namespace image_collector

#endif  // IMAGE_COLLECTOR__IMAGE_COLLECTOR_NODE_HPP_
