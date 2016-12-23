#include "geonav_transform/geonav_transform.h"
#include "geonav_transform/navsat_conversions.h"
#include "geonav_transform/geonav_utilities.h"

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

    // Load ROS parameters
    nh_priv.param("frequency", frequency, 10.0);
    nh_priv.getParam("magnetic_declination_radians", magnetic_declination_);
    nh_priv.param("yaw_offset", yaw_offset_, 0.0);
    nh_priv.param("broadcast_utm_transform", broadcast_utm_transform_, false);
    nh_priv.param("zero_altitude", zero_altitude_, false);
    nh_priv.param("publish_filtered_gps", publish_gps_, false);
    nh_priv.param("use_odometry_yaw", use_odometry_yaw_, false);
    nh_priv.param("delay", delay, 0.0);
    // Data parameter - required
    if (! nh_priv.hasParam("datum"))
    {
      ROS_ERROR("ERROR <datum> parameter is not supplied in "
		"geonav_transform configuration");
    }
    else
    {
      XmlRpc::XmlRpcValue datum_config;
      try
      {
	double datum_lat;
	double datum_lon;
	double datum_yaw;
	nh_priv.getParam("datum", datum_config);
	
	/* Handle datum specification. 
	   Users should always specify a baseLinkFrameId_ in the
	   datum config, but we had a release where it wasn't used, 
	   so we'll maintain compatibility.*/
	ROS_ASSERT(datum_config.getType() == XmlRpc::XmlRpcValue::TypeArray);
	ROS_ASSERT(datum_config.size() >= 3);
	
	if (datum_config.size() > 3)
	{
	  ROS_WARN_STREAM("Deprecated datum parameter configuration detected. "
			  "Only the first three parameters "
			  "(latitude, longitude, yaw) will be used. frame_ids "
			  "will be derived from odometry and navsat inputs.");
	}
	// Parse the data spec.
	std::ostringstream ostr;
	ostr << datum_config[0] << " " << datum_config[1] << " " << datum_config[2];
	std::istringstream istr(ostr.str());
	istr >> datum_lat >> datum_lon >> datum_yaw;
	
	// Try to resolve tf_prefix
	std::string tf_prefix = "";
	std::string tf_prefix_path = "";
	if (nh_priv.searchParam("tf_prefix", tf_prefix_path))
	{
	    nh_priv.getParam(tf_prefix_path, tf_prefix);
	}

	// Append the tf prefix in a tf2-friendly manner
	GeonavUtilities::appendPrefix(tf_prefix, world_frame_id_);
	GeonavUtilities::appendPrefix(tf_prefix, base_link_frame_id_);
	
	// Convert specified yaw to quaternion
	tf2::Quaternion quat;
	quat.setRPY(0.0, 0.0, datum_yaw);
	// Set datum
	setDatum(datum_lat,datum_lon,quat);
      }
      catch (XmlRpc::XmlRpcException &e)
      {
        ROS_ERROR_STREAM("ERROR reading sensor config: " << e.getMessage() <<
                         " for process_noise_covariance (type: " << datum_config.getType() << ")");
      }
    } // end of datum config.
    } // end of ::run()


    bool GeonavTransform::setDatum(double lat, double lon, tf2::Quaternion q)
    {
      
    }
} // namespace GeonavTransform
