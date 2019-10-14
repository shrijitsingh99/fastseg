//
// Created by shrijitsingh99 on 10/13/19.
//

#ifndef FAST_SEG_INCLUDE_FASTSEG_GROUNDPLANEFITTING_H_
#define FAST_SEG_INCLUDE_FASTSEG_GROUNDPLANEFITTING_H_

#include <pcl/common/centroid.h>
#include "fastseg/point_cloud.h"


namespace fastseg {

using namespace point_cloud;

class GroundPlaneFitting {
 public:
  GroundPlaneFitting(double sensor_height, double threshold_distance, int num_iterations, int num_lpr, double seed_threshold, double mirror_reflection_factor, bool lpr_estimation_mean);
  ~GroundPlaneFitting() = default;

  struct PlaneParameters {
    Eigen::MatrixXf normal;
    double threshold_dist_d;
    bool valid;
  };

  void extract_initial_seeds(pcl::PointCloud<PointXYZIRL> &point_cloud, pcl::PointCloud<PointXYZIRL> &seeds);
  void filter_mirror_reflection(pcl::PointCloud<PointXYZIRL> &point_cloud);
  PlaneParameters estimate_plane(pcl::PointCloud<PointXYZIRL> &point_cloud);
  void extract_ground(pcl::PointCloud<PointXYZIRL> &point_cloud, pcl::PointCloud<PointXYZIRL> &seeds, pcl::PointCloud<PointXYZIRL> &ground, pcl::PointCloud<PointXYZIRL> &not_ground);


 private:
  int num_lpr_;
  double seed_threshold_;
  bool lpr_estimation_mean_;
  double sensor_height_;
  double mirror_reflection_factor_;
  int num_iterations_;
  double threshold_distance_;
};
}
#endif //FAST_SEG_INCLUDE_FASTSEG_GROUNDPLANEFITTING_H_