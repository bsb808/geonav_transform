#ifndef GEONAV_TRANSFORM_FILTER_UTILITIES_H
#define GEONAV_TRANSFORM_FILTER_UTILITIES_H

#include <Eigen/Dense>

#include <iomanip>
#include <iostream>
#include <string>
#include <vector>


#define FB_DEBUG(msg) if (getDebug()) { *debugStream_ << msg; }
// Handy methods for debug output
std::ostream& operator<<(std::ostream& os, const Eigen::MatrixXd &mat);
std::ostream& operator<<(std::ostream& os, const Eigen::VectorXd &vec);
std::ostream& operator<<(std::ostream& os, const std::vector<size_t> &vec);
std::ostream& operator<<(std::ostream& os, const std::vector<int> &vec);

namespace GeonavTransform
{
  // from filter_common.h
  //! @brief Pose and twist messages each
  //! contain six variables
  const int POSE_SIZE = 6;
  const int TWIST_SIZE = 6;
  const int POSITION_SIZE = 3;
  const int ORIENTATION_SIZE = 3;
  const int ACCELERATION_SIZE = 3;
  
  //! @brief Common variables
  const double PI = 3.141592653589793;
  const double TAU = 6.283185307179587;


namespace GeonavUtilities
{

  //! @brief Utility method keeping RPY angles in the range [-pi, pi]
  //! @param[in] rotation - The rotation to bind
  //! @return the bounded value
  //!
  double clampRotation(double rotation);

  //! @brief Utility method for appending tf2 prefixes cleanly
  //! @param[in] tfPrefix - the tf2 prefix to append
  //! @param[in, out] frameId - the resulting frame_id value
  //!
  void appendPrefix(std::string tfPrefix, std::string &frameId);

}  // namespace GeonavUtilities
}  // namespace GeonavTranform

#endif  // ROBOT_LOCALIZATION_FILTER_UTILITIES_H
