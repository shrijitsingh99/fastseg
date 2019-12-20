//
// Created by shrijitsingh99 on 10/13/19.
//

#ifndef FASTSEG_INCLUDE_FASTSEG_FASTSEG_H_
#define FASTSEG_INCLUDE_FASTSEG_FASTSEG_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>

#include "ground_plane_fitting.h"
#include "curb_detection.h"

namespace fastseg {
class FastSeg {
 public:
  FastSeg();
  ~FastSeg() = default;

  void point_cloud_callback(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg);


  ros::NodeHandle nh;
  ros::Subscriber point_cloud_sub;
  ros::Publisher all_points_pub;
  ros::Publisher ground_points_pub;
  ros::Publisher no_ground_points_pub;
  ros::Publisher beam_marker_pub;

  GroundPlaneFitting *ground_plane_fit;
  CurbDetection *curb;

  pcl::PointCloud<PointXYZIRL> point_cloud;
  pcl::PointCloud<PointXYZIRL> ground_cloud;
  pcl::PointCloud<PointXYZIRL> no_ground_cloud;
  pcl::PointCloud<PointXYZIRL> seed_points;

};
}

#endif //FASTSEG_INCLUDE_FASTSEG_FASTSEG_H_
