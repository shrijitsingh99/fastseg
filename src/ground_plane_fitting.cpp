//
// Created by shrijitsingh99 on 10/13/19.
//

#include "fastseg/ground_plane_fitting.h"
#include "fastseg/point_cloud.h"

namespace fastseg {

using namespace point_cloud;

GroundPlaneFitting::GroundPlaneFitting(double sensor_height, double threshold_distance, int num_iterations, int num_lpr, double seed_threshold, double mirror_reflection_factor, bool lpr_estimation_mean):
sensor_height_(sensor_height), threshold_distance_(threshold_distance), num_iterations_(num_iterations), num_lpr_(num_lpr), seed_threshold_(seed_threshold), mirror_reflection_factor_(mirror_reflection_factor), lpr_estimation_mean_(lpr_estimation_mean) {

}

void GroundPlaneFitting::extract_initial_seeds(pcl::PointCloud<PointXYZIRL> &point_cloud, pcl::PointCloud<PointXYZIRL> &seeds) {
  int count = 0;
  double lpr_height = 0;
  seeds.clear();
  if (lpr_estimation_mean_) {
    double sum = 0.0;
    for (auto point = point_cloud.points.begin(); count < num_lpr_ && point < point_cloud.points.end(); ++point) {
      if (!isnan(point->x) && !isnan(point->y) && !isnan(point->z)) {
        sum += point->z;
        count++;
      }
    }
    lpr_height = (count != 0) ? (sum / count) : 0.0;
  } else {
    pcl::PointCloud<PointXYZIRL> temp_points;
    for (auto point = point_cloud.points.begin(); count < num_lpr_ && point < point_cloud.points.end(); ++point) {
      if (!isnan(point->x) && !isnan(point->y)  && !isnan(point->z)) {
        temp_points.points.push_back(*point);
        count++;
      }
    }
    point_cloud::sort(temp_points, Z);
    if (!temp_points.points.empty())
      lpr_height = (count % 2 == 0) ? (temp_points.points[count / 2 - 1].z + temp_points.points[count / 2 ].z) / 2 : temp_points.points[count / 2].z;
  }


  for (auto point = point_cloud.points.begin(); point < point_cloud.points.end(); ++point) {
    if (point->z < lpr_height + seed_threshold_) {
      seeds.points.push_back(*point);
    }
  }
}

void GroundPlaneFitting::filter_mirror_reflection(pcl::PointCloud<PointXYZIRL> &point_cloud) {
  auto point = point_cloud.points.begin();
  for(; point < point_cloud.points.end(); ++point){
    if(point->z > -1 * mirror_reflection_factor_* sensor_height_)
      break;
  }
  point_cloud.points.erase(point_cloud.begin(), point);
}

GroundPlaneFitting::PlaneParameters GroundPlaneFitting::estimate_plane(pcl::PointCloud<PointXYZIRL> &point_cloud) {
  PlaneParameters params = PlaneParameters();
  params.valid = false;

  if (point_cloud.points.empty())
    return params;

  Eigen::Matrix3f cov;
  Eigen::Vector4f pc_mean;
  pcl::computeMeanAndCovarianceMatrix(point_cloud, cov, pc_mean);

  // Singular Value Decomposition: SVD
  Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
  // use the least singular vector as normal
  params.normal = (svd.matrixU().col(2));
  // mean ground seeds value
  Eigen::Vector3f seeds_mean = pc_mean.head<3>();

  // according to normal.T*[x,y,z] = -d
  double d = -(params.normal.transpose() * seeds_mean)(0, 0);
  // set distance threshold to `th_dist - d`
  params.threshold_dist_d = threshold_distance_ - d;
  params.valid = true;

  return params;
}

void GroundPlaneFitting::extract_ground(pcl::PointCloud<PointXYZIRL> &point_cloud,  pcl::PointCloud<PointXYZIRL> &seeds, pcl::PointCloud<PointXYZIRL> &ground, pcl::PointCloud<PointXYZIRL> &not_ground) {
  ground = seeds;
  seeds.clear();
  for (int i = 0; i < num_iterations_; ++i) {
    PlaneParameters params = estimate_plane(ground);

    ground.clear();
    not_ground.clear();

    if (!params.valid)
      break;

    // Co
    Eigen::MatrixXf points(point_cloud.points.size(),3);
    int j =0;
    for(auto point : point_cloud.points){
      points.row(j++) << point.x, point.y, point.z;
    }
    // ground plane model
    Eigen::VectorXf model = points * params.normal;

    // threshold filter
    for(int row = 0; row < model.rows(); row++){
      if(model[row] < params.threshold_dist_d){
        point_cloud.points[row].label = 1u;// means ground
        ground.points.push_back(point_cloud.points[row]);
      } else{
        point_cloud.points[row].label = 0u;// means not ground and non clusterred
        not_ground.points.push_back(point_cloud.points[row]);
      }
    }
  }
}


}