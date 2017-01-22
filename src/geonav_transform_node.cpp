#include "geonav_transform/geonav_transform.h"

#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "geonav_transform_node");
  GeonavTransform::GeonavTransform trans;
  trans.run();
  return 0;
}
