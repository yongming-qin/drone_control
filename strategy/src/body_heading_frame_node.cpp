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
  ros::init(argc, argv, "body_heading_frame");
  ros::NodeHandle nh;

  std::string baseLink;
  nh.getParam("base_link", baseLink, "/base_link");

  tf::TransformBroadcaster broadcaster;
  tf::TransformListener listener;
  tf::Transform transformBroadcaster;
  tf::StampedTransform transformListener;


  ros::Rate rate(1000.0);
  while (nh.ok()) {
    try {
      listener.lookupTransform("/map", baseLink, ros::Time(0), transformListener);
      transformBroadcaster.setOrigin(transformListener.getOrigin());
      transformBroadcaster.setRotation();
      broadcaster.sendTransform(tf::StampedTransform(transformBroadcaster, , baseLink, "/heading_link"));
    }
    catch (tf::TransformException ex) {
      ROS_DEBUG_STREAM(ex.what());
    }

    rate.sleep();
  }
  return 0;

}