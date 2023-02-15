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

#include "image_collector/image_collector.hpp"

#include <iostream>

namespace image_collector
{

ImageCollector::ImageCollector()
{
}

void ImageCollector::setParameters(std::string save_format, std::string save_dir)
{
  if (save_format != "png" && save_format != "jpg" && save_format != "tiff") {
    std::cout << "Wrong save format. Using default: png" << std::endl;
    save_format = "png";
  } else {
    std::cout << "Using save format: " << save_format << std::endl;
    save_format_ = save_format;
  }
  if (save_dir == "") {
    std::cout << "No save directory provided. Using default: ./camera_data" << std::endl;
    save_dir_ = std::filesystem::current_path() / std::filesystem::path("camera_data");
  } else {
    std::cout << "Using save directory: " << save_dir << std::endl;
    save_dir_ = std::filesystem::path(save_dir);
  }
  std::filesystem::create_directories(save_dir_);
}

void ImageCollector::saveImages(const cv::Mat & img_rgb, const cv::Mat & img_depth)
{
  auto stamp = std::to_string(
    std::chrono::duration_cast<std::chrono::duration<double>>(
      std::chrono::system_clock::now().time_since_epoch()).count());

  auto rgb_path = std::filesystem::path(save_dir_) / std::filesystem::path(
    stamp + "_rgb." + save_format_);

  auto depth_path = std::filesystem::path(save_dir_) / std::filesystem::path(
    stamp + "_depth.tiff");

  cv::imwrite(rgb_path, img_rgb);
  cv::imwrite(depth_path, img_depth);
}
}  // namespace image_collector
