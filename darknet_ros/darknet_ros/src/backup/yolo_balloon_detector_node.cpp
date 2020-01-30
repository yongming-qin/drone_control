/*
 * yolo_balloon_detector_node.cpp
 *
 * Yongming Qin
 * 2020/01/21
 */

#include <darknet_ros/YoloBalloonPosition.hpp>
#include <ros/ros.h>

int main(int argc, char** argv) {
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
    ros::console::notifyLoggerLevelsChanged();
  }
  ros::init(argc, argv, "darknet_ros");
  ros::NodeHandle nodeHandle("~");
  balloon_ros::YoloObjectDetector yoloObjectDetector(nodeHandle);

  return 0;
}
