/*
 * YoloBalloonPosition.cpp
 * Subscribe to the image and the point cloud from zed camera using message_filter
 * Get the ROI box from YOLO can look up the position from the point cloud data.
 * Yongming Qin
 * 2020/01/19
 * 2020/01/26 add publisher
 */

// yolo object detector
#include "darknet_ros/YoloBalloonPosition.hpp"

// Check for xServer
#include <X11/Xlib.h>



namespace balloon_ros {
#ifdef DARKNET_FILE_PATH
std::string darknetFilePath_ = DARKNET_FILE_PATH;
#else
#error Path of darknet repository is not defined in CMakeLists.txt.
#endif

char *cfg;
char *weights;
char *data;
char **detectionNames;

YoloObjectDetector::YoloObjectDetector(ros::NodeHandle nh)
    : nodeHandle_(nh),
      numClasses_(0),
      classLabels_(0)
{
  ROS_INFO("[YoloObjectDetector] Node started.");

  // Read parameters from config file.
  if (!readParameters()) {
    ros::requestShutdown();
  }

  init();
}


bool YoloObjectDetector::readParameters()
{
  // Set vector sizes.
  nodeHandle_.param("yolo_model/detection_classes/names", classLabels_,
                    std::vector<std::string>(0));
  numClasses_ = classLabels_.size();
  return true;
}

void YoloObjectDetector::init()
{
  ROS_INFO("[YoloObjectDetector] init().");

  // Initialize deep network of darknet.
  std::string weightsPath;
  std::string configPath;
  std::string dataPath;
  std::string configModel;
  std::string weightsModel;

  float thresh; // Threshold of object detection.
  nodeHandle_.param("yolo_model/threshold/value", thresh, (float) 0.3);

  nodeHandle_.param("n_points", nPoints_, 20);
  nodeHandle_.param("lookup_limits", lookupLimits_, 10);
  nodeHandle_.param("in_box_scale", inBoxScale_, (float) 0.8);
  nodeHandle_.param("search_radius", searchRadius_, (float) 0.5);
  nodeHandle_.param("min_neighbors", minNeighbors_, 10);
  
  // Path to weights file.
  nodeHandle_.param("yolo_model/weight_file/name", weightsModel,
                    std::string("yolov2-tiny.weights"));
  nodeHandle_.param("weights_path", weightsPath, std::string("/default"));
  weightsPath += "/" + weightsModel;
  weights = new char[weightsPath.length() + 1];
  strcpy(weights, weightsPath.c_str());

  // Path to config file.
  nodeHandle_.param("yolo_model/config_file/name", configModel, std::string("yolov2-tiny.cfg"));
  nodeHandle_.param("config_path", configPath, std::string("/default"));
  configPath += "/" + configModel;
  cfg = new char[configPath.length() + 1];
  strcpy(cfg, configPath.c_str());

  // Path to data folder.
  dataPath = darknetFilePath_;
  dataPath += "/data";
  data = new char[dataPath.length() + 1];
  strcpy(data, dataPath.c_str());

  // Get classes.
  detectionNames = (char**) realloc((void*) detectionNames, (numClasses_ + 1) * sizeof(char*));
  for (int i = 0; i < numClasses_; i++) {
    detectionNames[i] = new char[classLabels_[i].length() + 1];
    strcpy(detectionNames[i], classLabels_[i].c_str());
  }

  // Load network.
  setupNetwork(cfg, weights, data, thresh, detectionNames, numClasses_,
                0, 0, 1, 0.5, 0, 0, 0, 0);

  // Initialize publisher and subscriber.
  std::string cameraTopicName;
  int cameraQueueSize;
  std::string pcTopicName;
  int pcQueueSize;

  nodeHandle_.param("subscribers/camera_reading/topic", cameraTopicName,
                    std::string("/zed/zed_node/left/image_rect_color"));
  ROS_INFO_STREAM("cameraTopicName: " << cameraTopicName);
  nodeHandle_.param("subscribers/camera_reading/queue_size", cameraQueueSize, 1);
  nodeHandle_.param("subscribers/pointcloud/topic", pcTopicName,
                    std::string("/zed/zed_node/point_cloud/cloud_registered"));
  ROS_INFO_STREAM("pcTopicName: " << pcTopicName);
  nodeHandle_.param("subscribers/pointcloud/queue_size", pcQueueSize, 1);

  //------------------------------------message_filters subscriber--------------------------------------
  message_filters::Subscriber<sensor_msgs::Image> imageSubscriber(nodeHandle_, cameraTopicName, cameraQueueSize);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pcSubscriber(nodeHandle_, pcTopicName, pcQueueSize);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), imageSubscriber, pcSubscriber);
  sync.registerCallback(boost::bind(&YoloObjectDetector::callback, this, _1, _2) );
 
  { // yolo()
    int i;
    demoTotal_ = sizeNetwork(net_);
    predictions_ = (float **) calloc(demoFrame_, sizeof(float*));
    for (i = 0; i < demoFrame_; ++i) {
      predictions_[i] = (float *) calloc(demoTotal_, sizeof(float));
    }
    avg_ = (float *) calloc(demoTotal_, sizeof(float));
    
    layer l = net_->layers[net_->n - 1];
    roiBoxes_ = (balloon_ros::RosBox_ *) calloc(l.w * l.h * l.n, sizeof(balloon_ros::RosBox_));
    ROS_INFO_STREAM("dimension: w: " << l.w << " h: " << l.h << " n: " << l.n);
  }

  //----------------------------------------publisher-------------------------------------------
  std::string balloonPositionsTopicName;
  nodeHandle_.param("publishers/balloon_positions/topic", balloonPositionsTopicName,
          std::string("/darknet_ros/balloon_positions"));
  int balloonPositionsQueueSize;
  nodeHandle_.param("subscribers/balloon_positions/queue_size", balloonPositionsQueueSize, 1);
  balloonPositionsPublisher_ = nodeHandle_.advertise<darknet_ros_msgs::BalloonPositions>(balloonPositionsTopicName, balloonPositionsQueueSize);

  ros::Time t;
  t = t.now();
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return;
}

void YoloObjectDetector::callback(const sensor_msgs::Image::ConstPtr& image,
        const sensor_msgs::PointCloud2::ConstPtr& pc)
{
  ROS_DEBUG("[YoloObjectDetector] ZED image and pointcloud received.");
  
  static cv_bridge::CvImagePtr cam_image;
  static geometry_msgs::PointStamped pointStamped, pointStampedMap;

  try { // cv
    cam_image = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  if (cam_image) {
    imageHeader_ = image->header;
    camImageCopy_ = cam_image->image.clone();
    frameWidth_ = cam_image->image.size().width;
    frameHeight_ = cam_image->image.size().height;
    ROS_DEBUG_STREAM("frameWidth_: " << frameWidth_ << " frameHeight_: " << frameHeight_);
  }

  { // fetchInThread()
    IplImage* ROS_img = new IplImage(camImageCopy_);
    if (FIRST_TIME_) {
      // from yolo(). ipl_to_image() seems do something important compared with ipl_into_image()
      // and it allocates memory. Repeatedly using this will cause memory explosion.
      buff_[0] = ipl_to_image(ROS_img);
      buffLetter_[0] = letterbox_image(buff_[0], net_->w, net_->h);
      FIRST_TIME_ = false;
    } else {
      ipl_into_image(ROS_img, buff_[buffIndex_]);
      rgbgr_image(buff_[buffIndex_]);  //QIN not sure what's this for
      letterbox_image_into(buff_[buffIndex_], net_->w, net_->h, buffLetter_[buffIndex_]);
    }
  }

  { // detectInThread()
    layer l = net_->layers[net_->n - 1];
    float *X = buffLetter_[0].data;
    float *prediction = network_predict(net_, X);

    rememberNetwork(net_); //TODO: may be deleted.
    detection *dets = 0;
    int nboxes = 0;
    dets = avgPredictions(net_, &nboxes);
    float nms = .4; //QIN: What does this mean?
    if (nms > 0) do_nms_obj(dets, nboxes, l.classes, nms);

    //QIN Look up the point cloud data, and estimate the average position of the box
    int i, j, n_points = 0;
    int count = 0;
    ROS_DEBUG_STREAM("nboxes: " << nboxes);
    for (i = 0; i < nboxes; ++i) {
      float x = dets[i].bbox.x, y = dets[i].bbox.y;
      float w = dets[i].bbox.w, h = dets[i].bbox.h;
      ROS_DEBUG_STREAM("x: " << x << " y: " << y << " w: " << w << " h: " << h);

      // BoundingBox must be 2% size of frame
      if (w > 0.02 && h > 0.02) {
        darknet_ros_msgs::BalloonPosition balloonPosition;
        int num_prob = 0; // for DEBUG:
        for (j = 0; j < demoClasses_; ++j) {
          if (dets[i].prob[j]) { // probability of this class QIN: are there cases more than 1? ONLY one.
            ROS_DEBUG_STREAM("In callback, number of probaility that is not 0: " << ++num_prob);
            n_points = getPositionPointcloud(pc, x, y, w, h); // put result in centroid_
            ROS_DEBUG_STREAM("class: " << j << "; prob: " << dets[i].prob[j]);

            if (n_points > nPoints_) { //QIN: Ensure there is enough cloud points. The number is my guess.
              count++;
              // Frame transfromation
              //QIN. image is of zed_left_camera_optical_frame; pc is of zed_left_camera_frame.
              // They are different wrt orientation.
              pointStamped.header = pc->header;
              pointStamped.point.x = centroid_[0];
              pointStamped.point.y = centroid_[1];
              pointStamped.point.z = centroid_[2];
              for (int i = 0; i < lookupLimits_; ++i) { //QIN
                try {tf_.transformPoint("/map", pointStamped, pointStampedMap);}
                catch (tf::TransformException &ex) {
                  // ROS_INFO_STREAM(ex.what() << "\n" << i);
                  ROS_INFO_STREAM("lookup: " << i);
                  continue;
                }
                break;
              }
              
              
              ROS_DEBUG_STREAM(pointStamped);
              // Message
              balloonPosition.Class = classLabels_[j];
              balloonPosition.probability = dets[i].prob[j];
              balloonPosition.distance_to_drone = sqrt(pow(centroid_[0],2) + pow(centroid_[1],2) + pow(centroid_[2],2));
              balloonPosition.point = pointStampedMap.point;
              balloonPositionsResults_.balloon_positions.push_back(balloonPosition);
            }
          }
        }
      }
    }
    // If no balloon position is estimated 
    balloonPositionsResults_.exist = (count > 0 ? true : false);
    balloonPositionsResults_.header = pc->header; //QIN
    balloonPositionsResults_.header.frame_id = "/map";
    balloonPositionsPublisher_.publish(balloonPositionsResults_);

    free_detections(dets, nboxes);
    balloonPositionsResults_.balloon_positions.clear();
  }
  ROS_DEBUG_STREAM("End of callback.");
  return;
}



int YoloObjectDetector::getPositionPointcloud(const sensor_msgs::PointCloud2ConstPtr &pc,
        float x, float y, float w, float h)
{
  // pcl conversion
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*pc, *cloud);

  int xmin = (x - w * inBoxScale_ / 2) * frameWidth_; //QIN
  int ymin = (y - h * inBoxScale_ / 2) * frameHeight_;
  int xmax = (x + w * inBoxScale_ / 2) * frameWidth_;
  int ymax = (y + h * inBoxScale_ / 2) * frameHeight_;

  if (xmin < 0) {xmin = 0;}
  if (ymin < 0) {ymin = 0;}
  if (xmax > frameWidth_) {xmax = frameWidth_;}
  if (ymax > frameHeight_) {ymax = frameHeight_;}

  // ROS_DEBUG_STREAM("min, max: " << xmin << " " << xmax << " " << ymin << " " << ymax);

  pcl::PointIndices::Ptr rectangleIndices(new pcl::PointIndices);
  for (int row = ymin; row < ymax; ++row) {
    for (int col = xmin; col < xmax; ++col) {
      rectangleIndices->indices.push_back(row * frameWidth_ + col);
    }
  }

  // ExtractIndices filter
  pcl::ExtractIndices<pcl::PointXYZ> filterExtractIndices;
  filterExtractIndices.setInputCloud(cloud);
  filterExtractIndices.setIndices(rectangleIndices);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFilteredIndices(new pcl::PointCloud<pcl::PointXYZ>);
  filterExtractIndices.filter(*cloudFilteredIndices);

  // Remove NaN
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloudFilteredIndices, *cloudFilteredIndices, indices);

  // Filter
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> radiusOutlierRemoval;
  radiusOutlierRemoval.setInputCloud(cloudFilteredIndices);
  radiusOutlierRemoval.setRadiusSearch(searchRadius_); //QIN
  radiusOutlierRemoval.setMinNeighborsInRadius(minNeighbors_); //QIN
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFilteredRadius(new pcl::PointCloud<pcl::PointXYZ>);
  radiusOutlierRemoval.filter(*cloudFilteredRadius);

  pcl::compute3DCentroid(*cloudFilteredRadius, centroid_);
  return cloudFilteredRadius->points.size();
}

void YoloObjectDetector::setupNetwork(char *cfgfile, char *weightfile, char *datafile, float thresh,
                                      char **names, int classes,
                                      int delay, char *prefix, int avg_frames, float hier, int w, int h,
                                      int frames, int fullscreen)
{
  demoPrefix_ = prefix;
  demoDelay_ = delay;
  demoFrame_ = avg_frames;
  image **alphabet = load_alphabet_with_file(datafile);
  demoNames_ = names;
  demoAlphabet_ = alphabet;
  demoClasses_ = classes;
  demoThresh_ = thresh;
  demoHier_ = hier;
  fullScreen_ = fullscreen;
  printf("YOLO V3\n");
  net_ = load_network(cfgfile, weightfile, 0);
  set_batch_network(net_, 1);
}


int YoloObjectDetector::sizeNetwork(network *net)
{
  int i;
  int count = 0;
  for(i = 0; i < net->n; ++i){
    layer l = net->layers[i];
    if(l.type == YOLO || l.type == REGION || l.type == DETECTION){
      count += l.outputs;
    }
  }
  return count;
}

void YoloObjectDetector::rememberNetwork(network *net)
{
  int i;
  int count = 0;
  for(i = 0; i < net->n; ++i){
    layer l = net->layers[i];
    if(l.type == YOLO || l.type == REGION || l.type == DETECTION){
      memcpy(predictions_[demoIndex_] + count, net->layers[i].output, sizeof(float) * l.outputs);
      count += l.outputs;
    }
  }
}

detection *YoloObjectDetector::avgPredictions(network *net, int *nboxes)
{
  int i, j;
  int count = 0;
  fill_cpu(demoTotal_, 0, avg_, 1);
  for(j = 0; j < demoFrame_; ++j){
    axpy_cpu(demoTotal_, 1./demoFrame_, predictions_[j], 1, avg_, 1);
  }
  for(i = 0; i < net->n; ++i){
    layer l = net->layers[i];
    if(l.type == YOLO || l.type == REGION || l.type == DETECTION){
      memcpy(l.output, avg_ + count, sizeof(float) * l.outputs);
      count += l.outputs;
    }
  }
  detection *dets = get_network_boxes(net, buff_[0].w, buff_[0].h, demoThresh_, demoHier_, 0, 1, nboxes);
  return dets;
}


} /* namespace ballon_ros*/
