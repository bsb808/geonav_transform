CURRENTLY UNDER DEVELOPMENT!

# geonav_transform

The goal of this package is to simplify the integration of accurate/precise geographic navigation information (typically from a sensor) into the ROS localization and navigation workflows.  To those ends, the geonav_tranform_node can perform the following functions:

  * Takes incoming Odometry messages, typically from a sensor, that contain a geographic position and sensor-frame orientation and velocities. (Note, it would be nice to have a new message the specifies this type of input - using Odometry is a bit of a hack.) 
  * Tranforms these message to new Odometry message that express the information in the following frames
    * utm
    * odom 
  * Broadcasts the following tf2 tranforms
    * utm->odom
    * odom->base_link
    
The use-case that motivated this project is integrating sensors that provide a GPS-aided INS solution (e.g., microstrain, advanced navigation, Xsens, etc.).  This situation is analogous to using an ekf/ukf node from robot_localization package to fuse IMU and GPS information, but in this case the processing is done by the sensor.  The purpose of this package is to allow integration of this type of sensor directly into the ROS navigation stack.
     
## Parameters

  * ~datum: The origin of the local "odom" frame specified as a three element array [Latitude, Logitude, Altitude].  Lat/Lon are in decimal degrees; altitude is in meters.  Default is [0.0 0.0 0.0] and this is probably not what you want!
  * ~frequency: The frequency of broadcasting the tf2 tranforms.  The Odometry messages are published at the same rate as the incoming Odometry messages.  Default is 10 Hz
  * ~broadcast_utm2odom_transform: Whether or not to broadcast the utm->odom tranform.  Default is True.
  * ~broadcast_odom2base_transform: Whether or not to broadcast the odom->base_link tranform.  Default is True.
  * ~zero_altitude
  * ~base_link_frame_id: Default is "base_link"
  * ~odom_frame_id: Default is "odom"
  * ~utm_frame_id: Default is "utm"


## Subscribed Topics

  * /odometry/nav:  A nav_msgs/Odometry message with geographic position and velocity data.  The message is organized as follows:
    * The header.frame_id and child_frame_id values are ignored.
    * pose.pose.position is
      * .y = Latitude [dec. degrees]
      * .x = Longitude [dec. degrees]
      * .z = Altitude [m]
    * pose.pose.orientation of the base_link relative to a fixed ENU coordinate frame
      * If the ~orientation_ned parameter is set to true, the node will convert the orientation from NED to ENU.
      * For now we are assuming the orientation is true (not magnetic).  Typically the magnetic declination will be set internal to the sensor providing the information.
    * pose.covariance is expressed in meters for position and radians for orientation (REP-103)
    * twist.twist.linear/angular is the velocity in the base_link frame
    * twist.covariance is expressed in m/s and rad/s.
      
  
## Published Topics

The following message are published at the same rate as incoming /odometry/nav messages.  Consistent with the navigation stack and robot_localization, the Odometry messages contain position pose information relative to the fixed frame (utm or odom, reported as the header.frame_id).  The velocity (twist) information is relative to the mobile frame (base_link, reported as the .child_frame).

  * /odometry/odom:  A nav_msgs/Odometry message in the local odom frame (relative to the datum)
  * /odometry/utm:   A nav_msgs/Odometry message in the UTM frame
    * This is published as a static tranform: http://wiki.ros.org/tf2_ros  http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20static%20broadcaster%20(C%2B%2B)

## Published Transforms

  * utm->odom
  * odom->base_link

## Frames

  * utm: The global UTM coordinate frame.  The origin of this frame (which UTM zone we are in) is determined by the datum parameter
  * odom: The local, fixed odom frame has an orgin specified by the datum parameter.  We have assumed that there is no orientation between UTM and the odom frame.  While this is not as general as possible, it simplifies the implementation, usage and interpretation.
  * base_link: This mobile frame typically coincides with the sensor frame.
