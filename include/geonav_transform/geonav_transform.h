
#ifndef GEONAV_TRANSFORM_GEONAV_TRANSFORM_H
#define GEONAV_TRANSFORM_GEONAV_TRANSFORM_H

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>

#include <string>

namespace GeonavTransform
{

class GeonavTransform
{
  public:
    //! @brief Constructor
    //!
    GeonavTransform();

    //! @brief Destructor
    //!
    ~GeonavTransform();

    //! @brief Main run loop
    //!
    void run();

  private:
    //! @brief Computes the transform from the UTM frame to the odom frame
    //!
    void computeTransformOdom2Utm();

    //! @brief Sets datum values
    //! yaw is ENU
    //!
    bool setDatum(double lat, double lon, double alt, tf2::Quaternion q);

    //! @brief Given the pose of the navsat sensor in the UTM frame, removes the offset from the vehicle's centroid
    //! and returns the UTM-frame pose of said centroid.
    //!
    void getRobotOriginUtmPose(const tf2::Transform &gps_utm_pose,
                               tf2::Transform &robot_utm_pose,
                               const ros::Time &transform_time);

    //! @brief Given the pose of the navsat sensor in the world frame, removes the offset from the vehicle's centroid
    //! and returns the world-frame pose of said centroid.
    //!
    void getRobotOriginWorldPose(const tf2::Transform &gps_odom_pose,
                                 tf2::Transform &robot_odom_pose,
                                 const ros::Time &transform_time);


    //! @brief Callback for the geo nav odom data
    //! @param[in] msg The odometry message to process
    //!
    void navOdomCallback(const nav_msgs::OdometryConstPtr& msg);

    //! @brief Converts the odometry data back to GPS and broadcasts it
    //! @param[out] filtered_gps The NavSatFix message to prepare
    //!
    bool prepareFilteredGps(sensor_msgs::NavSatFix &filtered_gps);

    //! @brief Prepares the GPS odometry message before sending
    //! @param[out] gps_odom The odometry message to prepare
    //!
    bool prepareGpsOdometry(nav_msgs::Odometry &gps_odom);

    //! @brief Used for setting the GPS data that will be used to compute the transform
    //! @param[in] msg The NavSatFix message to use in the transform
    //!
    void setTransformGps(const sensor_msgs::NavSatFixConstPtr& msg);

    //! @brief Used for setting the odometry data that will be used to compute the transform
    //! @param[in] msg The odometry message to use in the transform
    //!
    void setTransformOdometry(const nav_msgs::OdometryConstPtr& msg);

    //! @brief Frame ID of the robot's body frame
    //!
    //! This is needed for obtaining transforms from the robot's body frame to the frames of sensors (IMU and GPS)
    //!
    std::string base_link_frame_id_;

    //! @brief Whether or not we broadcast the UTM transform
    //!
    bool broadcast_utm_transform_;

    //! @brief The frame_id of the NAV message (specifies mounting location)
    //!
    std::string nav_frame_id_;

    //! @brief Timestamp of the latest good GPS message
    //!
    //! We assign this value to the timestamp of the odometry message that we output
    //!
    ros::Time gps_update_time_;

    //! @brief Whether or not we have new GPS data
    //!
    //! We only want to compute and broadcast our transformed GPS data if it's new. This variable keeps track of that.
    //!
    bool gps_updated_;

    //! @brief Whether or not the datam was set
    //!
    bool has_datum_;

    //! @brief Whether or not the GPS fix is usable
    //!
    bool has_transform_gps_;

    //! @brief Signifies that we have received a usable IMU message
    //!
    bool has_transform_imu_;

    //! @brief Signifies that we have received a usable odometry message
    //!
    bool has_transform_odom_;

    //! @brief Covariance for most recent odometry data
    //!
    Eigen::MatrixXd latest_odom_covariance_;

    //! @brief Covariance for most recent GPS/UTM data
    //!
    Eigen::MatrixXd latest_utm_covariance_;
    Eigen::MatrixXd geonav_utm_covariance_;

    //! @brief Latest GPS data, stored as UTM coords
    //!
    tf2::Transform latest_utm_pose_;
    tf2::Transform transform_utm2nav_;

    //! @brief Latest odometry pose data
    //!
    tf2::Transform latest_world_pose_;

    //! @brief Parameter that specifies the magnetic declination for the robot's environment.
    //!
    double magnetic_declination_;

    //! @brief Timestamp of the latest good odometry message
    //!
    //! We assign this value to the timestamp of the odometry message that we output
    //!
    ros::Time odom_update_time_;

    //! @brief Whether or not we have new odometry data
    //!
    //! If we're creating filtered GPS messages, then we only want to broadcast them when new odometry data arrives.
    //!
    bool odom_updated_;

    //! @brief Whether or not we publish filtered GPS messages
    //!
    bool publish_gps_;

    //! @brief Transform buffer for managing coordinate transforms
    //!
    tf2_ros::Buffer tf_buffer_;

    //! @brief Transform listener for receiving transforms
    //!
    tf2_ros::TransformListener tf_listener_;

    //! @brief Whether or not we've computed a good heading
    //!
    bool transform_good_;

    //! @brief Latest IMU orientation
    //!
    tf2::Quaternion transform_orientation_;

    //! @brief Holds the UTM pose that is used to compute the transform
    //!
    tf2::Transform transform_utm_pose_;


    //! @brief Latest IMU orientation
    //!
    tf2::Transform transform_world_pose_;

    //! @brief Whether we get the transform's yaw from the odometry or IMU source
    //!
    bool use_odometry_yaw_;


    //! @brief Used for publishing the static world_frame->utm transform
    //!
    tf2_ros::StaticTransformBroadcaster utm_broadcaster_;

    //! @brief Stores the yaw we need to compute the transform
    //!
    double utm_odom_tf_yaw_;

    //! @brief Holds the UTM->odom transform
    //!
    //tf2::Transform utm_world_transform_;
    tf2::Transform transform_utm2odom_;
    tf2::Transform transform_utm2odom_inverse_;

    //! @brief Holds the odom->UTM transform for filtered GPS broadcast
    //!
    tf2::Transform utm_world_trans_inverse_;

    //! @brief UTM zone as determined after transforming GPS message
    //!
    std::string utm_zone_;

    //! @brief Frame ID of the GPS odometry output
    //!
    //! This will just match whatever your odometry message has
    //!
    std::string world_frame_id_;

    //! @brief IMU's yaw offset
    //!
    //! Your IMU should read 0 when facing *magnetic* north. If it doesn't, this (parameterized) value gives the offset
    //! (NOTE: if you have a magenetic declination, use the parameter setting for that).
    //!
    double yaw_offset_;

    //! @brief Whether or not to report 0 altitude
    //!
    //! If this parameter is true, we always report 0 for the altitude of the converted GPS odometry message.
    //!
    bool zero_altitude_;

    //! @brief Publisher for Nav relative to odom (datum) frame
    ros::Publisher odom_pub_;
    //! @brief Publisher for Nav Odometry relative to utm frame
    ros::Publisher utm_pub_;
};

}  // namespace GeonavTransform

#endif  // GEONAV_TRANSFORM_NAVSAT_TRANSFORM_H
