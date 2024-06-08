#ifndef CSV2ROADIMG_CSV2ROADIMG_NODE_H
#define CSV2ROADIMG_CSV2ROADIMG_NODE_H

#include <deque>
#include <fstream>

#include "ros/node_handle.h"
#include "opencv4/opencv2/core.hpp"

namespace csv2roadimg
{

class Csv2roadimgNode : public ros::NodeHandle
{
public:
  Csv2roadimgNode();

private:
  static inline auto toMat(std::ifstream && ifs);

  cv::Mat roadimg_;
};

}

#endif