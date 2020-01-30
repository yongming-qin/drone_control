/*
 * yolo_balloon_detector_node.cpp
 *
 * Yongming Qin
 * 2020/01/21
 */

#include <darknet_ros/YoloBalloonPosition.hpp>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "darknet_ros");
  ros::NodeHandle nodeHandle("~");

  bool isDebug;
  nodeHandle.param("darknet_debug", isDebug, false);
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, 
          (isDebug ? ros::console::levels::Debug : ros::console::levels::Info)) )
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  balloon_ros::YoloObjectDetector yoloObjectDetector(nodeHandle);

  return 0;
}
