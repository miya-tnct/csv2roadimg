#include "ros/init.h"
#include "ros/spinner.h"

#include "csv2roadimg/csv2roadimg_node.h"

int main(int argc, char **argv)
{
  //ノードの初期化
  ros::init(argc, argv, "csv2roadimg");
  csv2roadimg::Csv2roadimgNode node;
  // ros::spin();
  return 0;
}