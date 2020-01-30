/**
 * subscribe to the topics of darknet_ros and based on the boxes
 * get the position in the cloud data.
 * Yongming Qin
 * 2020/01/16
 */

// c++
#include <math.h>
#include <string>
#include <vector>

// ros
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


int main(int argc, char** argv)
{

  ros::init(argc, argv, "position_pc");
  ros::NodeHandle nh;

  message_filters::Subscriber<sensor_msgs::Image> imageSubscriber(nh, cameraTopicName, cameraQueueSize);
  message_filters::Subscriber<sensor_msgs::Image> imageSubscriber(nh, cameraTopicName, cameraQueueSize);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pcSubscriber(nh, pcTopicName, pcQueueSize);


  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), imageSubscriber, pcSubscriber);
  sync.registerCallback(boost::bind(&YoloObjectDetector::callback, _1, _2));

  






}


