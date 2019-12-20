//
// Created by shrijitsingh99 on 10/13/19.
//

#include "fastseg/fastseg.h"

namespace fastseg {

FastSeg::FastSeg() {
  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

  int num_lpr, num_iterations, num_segments;
  double seed_threshold, threshold_distance, sensor_height, mirror_reflection_factor;
  bool lpr_estimation_mean;

  ros::NodeHandle nh_private("~");
  nh_private.param("num_lpr", num_lpr, 20);
  nh_private.param("seed_threshold", seed_threshold, 1.2);
  nh_private.param("lpr_estimation_mean", lpr_estimation_mean, true);
  nh_private.param("sensor_height", sensor_height, 1.9);
  nh_private.param("mirror_reflection_factor", mirror_reflection_factor, 1.5);
  nh_private.param("num_iterations", num_iterations, 10);
  nh_private.param("threshold_distance", threshold_distance, 0.3);
  nh_private.param("num_segments", num_segments, 1);
  // TODO (shrijitsingh99): Implement fitting of multiple planes

  // Ground Plane Fitting
  ground_plane_fit = new GroundPlaneFitting(sensor_height, threshold_distance, num_iterations, num_lpr, seed_threshold, mirror_reflection_factor, lpr_estimation_mean);

  // Curb Detection
  curb = new CurbDetection();

  all_points_pub =  nh.advertise<sensor_msgs::PointCloud2>("/all_points", 2);
  ground_points_pub = nh.advertise<sensor_msgs::PointCloud2>("/ground", 2);
  no_ground_points_pub = nh.advertise<sensor_msgs::PointCloud2>("/no_ground", 2);
  point_cloud_sub = nh.subscribe("cloud_in", 1,  &FastSeg::point_cloud_callback, this);
  beam_marker_pub =  nh.advertise<visualization_msgs::Marker>("/beams", 2);
}

void FastSeg::point_cloud_callback(const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg) {
  pcl::fromROSMsg(*point_cloud_msg, point_cloud);

  point_cloud::sort(point_cloud, Z);
  ground_plane_fit->filter_mirror_reflection(point_cloud);
  ground_plane_fit->extract_initial_seeds(point_cloud, seed_points);
  ground_plane_fit->extract_ground(point_cloud, seed_points, ground_cloud, no_ground_cloud);

  std::vector<PointXYZIRL> beams;
  std::vector<PointXYZIRL> directions;

  curb->sliding_beam_segmentation(point_cloud, ground_cloud, no_ground_cloud, beams, directions);


    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "/top_laser_link";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "beams";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.id = 2;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = 0.1;
    line_list.color.b = 1.0;
    line_list.color.a = 0.5;

    int i = 0;
    for (auto point : beams) {
        geometry_msgs::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = 0;
        line_list.points.push_back(p);
        ++i;
    }

    beam_marker_pub.publish(line_list);

    line_list.points.clear();

    for (auto point : directions) {
        geometry_msgs::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = 0.5;
        line_list.scale.x = 0.2;
        line_list.color.b = 0.0;
        line_list.color.g = 1.0;
        line_list.color.a = 1.0;
        line_list.points.push_back(p);
        ++i;
    }

    line_list.header.stamp = ros::Time::now();
    line_list.ns = "directions";
    beam_marker_pub.publish(line_list);

    sensor_msgs::PointCloud2 all_points_msg, ground_points_msg, no_ground_points_msg;

  point_cloud::set_attributes(point_cloud, 0, 0);
  point_cloud::set_attributes(ground_cloud, 0, 0);
  point_cloud::set_attributes(no_ground_cloud, 0, 0);

  pcl::toROSMsg(point_cloud, all_points_msg);
  pcl::toROSMsg(ground_cloud, ground_points_msg);
  pcl::toROSMsg(no_ground_cloud, no_ground_points_msg);

  all_points_msg.header.frame_id = ground_points_msg.header.frame_id = no_ground_points_msg.header.frame_id = point_cloud_msg->header.frame_id;
  all_points_msg.header.stamp = ground_points_msg.header.stamp = no_ground_points_msg.header.stamp = point_cloud_msg->header.stamp;

  all_points_pub.publish(all_points_msg);
  ground_points_pub.publish(ground_points_msg);
  no_ground_points_pub.publish(no_ground_points_msg);
}


}