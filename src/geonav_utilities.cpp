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

#include "geonav_transform/geonav_utilities.h"

//#include <string>
//#include <vector>

namespace GeonavTransform
{

namespace GeonavUtilities
{
  void appendPrefix(std::string tfPrefix, std::string &frameId)
  {
    // Strip all leading slashes for tf2 compliance
    if (!frameId.empty() && frameId.at(0) == '/')
    {
      frameId = frameId.substr(1);
    }

    if (!tfPrefix.empty() && tfPrefix.at(0) == '/')
    {
      tfPrefix = tfPrefix.substr(1);
    }

    // If we do have a tf prefix, then put a slash in between
    if (!tfPrefix.empty())
    {
      frameId = tfPrefix + "/" + frameId;
    }
  }

  double clampRotation(double rotation)
  {
    while (rotation > PI)
    {
      rotation -= TAU;
    }

    while (rotation < -PI)
    {
      rotation += TAU;
    }

    return rotation;
  }

}  // namespace GeonavUtilities

}  // namespace GeonavTransform
