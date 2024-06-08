#include "csv2roadimg/csv2roadimg_node.h"

#include <sstream>

#include "opencv4/opencv2/opencv.hpp"

namespace csv2roadimg
{

auto Csv2roadimgNode::toMat(std::ifstream && ifs)
{
  ROS_INFO("start convert csv");
  if (!ifs) {
    ROS_ERROR("faided to load csv");
    return cv::Mat();
  }
  std::deque<std::pair<double, double>> csv_info;
  std::string str_buf;
  while (ifs >> str_buf)
  {
    std::istringstream iss(str_buf);

    std::string time_str, range_str;
    if (!(getline(iss, time_str, ',') && getline(iss, range_str, ',')))
    {
      ROS_ERROR("cannot find lines");
      return cv::Mat();
    }

    double time, range;
    try {
      time = std::stod(time_str);
      range = std::stod(range_str);
    }
    catch (...) {
      ROS_ERROR("cannot convert string to double");
      return cv::Mat();
    }

    csv_info.emplace_back(time, range);
  }
  if (csv_info.empty()) {
    ROS_ERROR("no lines found");
    return cv::Mat();
  }

  auto resolution = 0.01;
  auto width = 2.0;
  auto road_width = 0.3;
  auto speed = 1.0;

  auto dissolution = 1.0 / resolution;
  int road_pixels = road_width * dissolution;

  auto roadimg = cv::Mat(
    csv_info.back().first * speed * dissolution,
    width * dissolution,
    CV_8UC3, cv::Vec3b(0, 0, 0));

  decltype(roadimg.rows) y = 0;
  for (const auto & csv_info_pair : csv_info) {
    auto road_centor_x = csv_info_pair.second * dissolution;
    auto road_l_x_range = std::make_pair(
      std::max<int>(0, road_centor_x - road_pixels),
      std::min<int>(road_centor_x + road_pixels, roadimg.cols));
    auto road_r_x_range = std::make_pair(
      roadimg.cols - 1 - road_l_x_range.second,
      roadimg.cols - 1 - road_l_x_range.first);
    auto y_max = csv_info_pair.first * speed * dissolution;
    while (y < y_max) {
      for (auto road_x_range : {road_l_x_range, road_r_x_range}) {
        auto ptr_zero = roadimg.ptr<cv::Vec3b>(y);
        auto ptr_end = &ptr_zero[road_x_range.second];
        for (auto ptr = &ptr_zero[road_x_range.first]; ptr < ptr_end; ++ptr) {
          (*ptr)[0] = 0xff;
          (*ptr)[1] = 0xff;
          (*ptr)[2] = 0xff;
        }
      }
      ++y;
    }
  }
  ROS_INFO("success convert csv");
  return roadimg;
}

Csv2roadimgNode::Csv2roadimgNode()
: ros::NodeHandle()
, roadimg_(toMat(std::ifstream("road.csv")))
{
  if (roadimg_.empty()) {
    return;
  }
  cv::imwrite("road.jpg", roadimg_);
}

}