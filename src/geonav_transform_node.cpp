/* 

Copyright (c) 2017, Brian Bingham
All rights reserved

This file is part of the geonav_transform package.

Geonav_transform is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Geonav_transform is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Foobar.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "geonav_transform/geonav_transform.h"

#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "geonav_transform_node");
  GeonavTransform::GeonavTransform trans;
  trans.run();
  return 0;
}
