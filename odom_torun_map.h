#ifndef __ODOM_DISPLAY_H__
#define __ODOM_DISPLAY_H__

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/LaserScan.h"
#include <sensor_msgs/PointCloud2.h>

#include "tf/tf.h"
#include "tf2/utils.h"
#include "tf/transform_listener.h"
#include "tf2_msgs/TFMessage.h"
#include "tf/transform_broadcaster.h"

#include <chrono>
#include <thread>

#include "./utils/common.h"

class OdomRunMap
{
public:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  std::string rosbag_path_;
  std::string rosbags_path_;

  std::string fusion_odom_topic_;
  std::string ground_truth_odom_topic_;
  std::string gnss_odom_in_map_topic_;

  OdomRunMap();
  ~OdomRunMap();

  void loadParam(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
  void readOdomFromRosbag();
  void baglistget();
};

#endif