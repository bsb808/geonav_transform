#include "geonav_transform/geonav_transform.h"
#include "geonav_transform/navsat_conversions.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <XmlRpcException.h>

#include <string>

namespace GeonavTransform
{
  GeonavTransform::GeonavTransform() :
    // Initialize attributes
    magnetic_declination_(0.0),
    utm_odom_tf_yaw_(0.0),
    yaw_offset_(0.0),
    broadcast_utm_transform_(false),
    has_transform_odom_(false),
    has_transform_gps_(false),
    has_transform_imu_(false),
    transform_good_(false),
    gps_frame_id_(""),
    gps_updated_(false),
    odom_updated_(false),
    publish_gps_(false),
    use_odometry_yaw_(false),
    use_manual_datum_(false),
    zero_altitude_(false),
    world_frame_id_("odom"),
    base_link_frame_id_("base_link"),
    utm_zone_(""),
    tf_listener_(tf_buffer_)
  {
    latest_utm_covariance_.resize(POSE_SIZE, POSE_SIZE);
    latest_odom_covariance_.resize(POSE_SIZE, POSE_SIZE);
  }

  GeonavTransform::~GeonavTransform()
  {
  }

  void GeonavTransform::run()
  {
    ros::Time::init();

    double frequency = 10.0;
    double delay = 0.0;

    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    // Load the parameters we need
    nh_priv.getParam("magnetic_declination_radians", magnetic_declination_);
    nh_priv.param("yaw_offset", yaw_offset_, 0.0);
    nh_priv.param("broadcast_utm_transform", broadcast_utm_transform_, false);
    nh_priv.param("zero_altitude", zero_altitude_, false);
    nh_priv.param("publish_filtered_gps", publish_gps_, false);
    nh_priv.param("use_odometry_yaw", use_odometry_yaw_, false);
    nh_priv.param("wait_for_datum", use_manual_datum_, false);
    nh_priv.param("frequency", frequency, 10.0);
    nh_priv.param("delay", delay, 0.0);
  } // end of ::run()
} // namespace GeonavTransform
