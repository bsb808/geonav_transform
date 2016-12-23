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
