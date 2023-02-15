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

#ifndef IMAGE_COLLECTOR__IMAGE_COLLECTOR_HPP_
#define IMAGE_COLLECTOR__IMAGE_COLLECTOR_HPP_

#include <cstdint>
#include <filesystem>
#include <string>

#include "opencv2/opencv.hpp"
#include "image_collector/visibility_control.hpp"


namespace image_collector
{

class IMAGE_COLLECTOR_PUBLIC ImageCollector
{
public:
  ImageCollector();
  void setParameters(std::string save_format, std::string save_dir);
  void saveImages(const cv::Mat & img_rgb, const cv::Mat & img_depth);

private:
  std::string save_format_{"png"};
  std::string save_dir_{""};
};

}  // namespace image_collector

#endif  // IMAGE_COLLECTOR__IMAGE_COLLECTOR_HPP_
