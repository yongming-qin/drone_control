/**
 * Add the body heading frame.
 * ENU: right: x , forward: y, upward: z
 * Yongming Qin
 * 2020/01/30
 * TODO: not finished
 */

#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "drone_withoutrotation_frame");
  ros::NodeHandle nh;

  std::string baseLink;
  nh.getParam("base_link", baseLink, "/base_link");

  tf::TransformBroadcaster broadcaster;
  tf::TransformListener listener;
  tf::StampedTransform tfBroadcast;
  tf::StampedTransform tfListen;


  ros::Rate rate(1000.0);
  while (nh.ok()) {
    try {
      listener.lookupTransform("/map", baseLink, ros::Time(0), tfListen);
      tfBroadcast.setOrigin( tfListen.getOrigin() );
      tf::Quaternion q = tfListen.getRotation();
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      
      double yaw = getYaw(q);
      tfBroadcast.setRotation( tf::Quaternion(0, 0, 0, 1) );
      broadcaster.sendTransform(tf::StampedTransform(broadcaster, tfListen.stamp_, /map, "/drone_without_rotation_frame"));
    }
    catch (tf::TransformException ex) {
      ROS_DEBUG_STREAM(ex.what());
    }

    rate.sleep();
  }
  return 0;

}