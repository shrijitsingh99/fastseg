//
// Created by shrijitsingh99 on 12/17/19.
//

#include "fastseg/curb_detection.h"
#include <pcl/filters/crop_box.h>

namespace fastseg {

    using namespace point_cloud;

    CurbDetection::CurbDetection() {
    }


    double zone_angle(double x, double y, double xb, double yb) {
        if (!isnan(x) && !isnan(y)) {
            double angle = atan2(y - yb, x - xb);
            if (angle < 0)
                angle = angle + 2 * M_PI;
            return angle;
        }
        return -1;
    }

    void CurbDetection::sliding_beam_segmentation(pcl::PointCloud<PointXYZIRL> &point_cloud,
                                                  pcl::PointCloud<PointXYZIRL> &ground_cloud,
                                                  pcl::PointCloud<PointXYZIRL> &no_ground_cloud,
                                                  std::vector<PointXYZIRL> &beams) {
        pcl::PointCloud<point_cloud::PointXYZIRL>::Ptr point_cloud_filtered(new pcl::PointCloud<point_cloud::PointXYZIRL>);
        pcl::PointCloud<point_cloud::PointXYZIRL>::Ptr ground_cloud_filtered(new pcl::PointCloud<point_cloud::PointXYZIRL>);
        pcl::PointCloud<point_cloud::PointXYZIRL>::Ptr no_ground_cloud_filtered(new pcl::PointCloud<point_cloud::PointXYZIRL>);

        pcl::CropBox<point_cloud::PointXYZIRL> boxFilter;
        boxFilter.setMin(Eigen::Vector4f(-30, -30, -5, 1.0));
        boxFilter.setMax(Eigen::Vector4f(30, 30, 5, 1.0));


        boxFilter.setInputCloud(pcl::PointCloud<point_cloud::PointXYZIRL>::Ptr(new pcl::PointCloud<point_cloud::PointXYZIRL>(point_cloud)));
        boxFilter.filter(*point_cloud_filtered);


        boxFilter.setInputCloud(pcl::PointCloud<point_cloud::PointXYZIRL>::Ptr(new pcl::PointCloud<point_cloud::PointXYZIRL>(ground_cloud)));
        boxFilter.filter(*ground_cloud_filtered);

        boxFilter.setInputCloud(pcl::PointCloud<point_cloud::PointXYZIRL>::Ptr(new pcl::PointCloud<point_cloud::PointXYZIRL>(no_ground_cloud)));
        boxFilter.filter(*no_ground_cloud_filtered);


        std::vector<std::vector<point_cloud::PointXYZIRL>> zones(360);
        for(auto point : (*point_cloud_filtered).points){

            double angle = zone_angle(point.x, point.y, 0, 0);
                if (angle >= 0) {
                    zones[int(angle *  180 / M_PI)].push_back(point);
                }
        }

        std::vector<std::vector<point_cloud::PointXYZIRL>> no_ground_zones(360);

        point_cloud::PointXYZIRL sensor_origin;
        sensor_origin.x = 0;
        sensor_origin.y = 0;
        sensor_origin.z = 0;
        sensor_origin.intensity = 255;
        sensor_origin.ring = 0;
        sensor_origin.label = 0;

        point_cloud::PointXYZIRL edge_point;
        sensor_origin.x = 0;
        sensor_origin.y = 0;
        sensor_origin.z = 0;
        sensor_origin.intensity = 255;
        sensor_origin.ring = 0;
        sensor_origin.label = 0;

        for(auto point : (*no_ground_cloud_filtered).points){
            double angle = zone_angle(point.x, point.y, 0, 0);
            if (angle >= 0) {
                no_ground_zones[int(angle *  180 / M_PI)].push_back(point);
            }
        }
        std::vector<point_cloud::PointXYZIRL> beam_min_point(360);

        int i = 0;
        bool clear = true;
        for (auto zone : no_ground_zones) {
            double min_dk = 100.0;
            for (auto point : zone) {
                if (pow(pow(point.x, 2) + pow(point.y, 2), 0.5) < min_dk) {
                    min_dk = pow(pow(point.x, 2) + pow(point.y, 2), 0.5);
                    beam_min_point[i] = point;
                    clear = false;
                }
            }
            beams.push_back(sensor_origin);
            if (!clear)
             beams.push_back(beam_min_point[i]);
            else {
                edge_point.x = 100 * cos(i * M_PI / 180);
                edge_point.y = 100 * sin(i * M_PI / 180);
                beams.push_back(edge_point);
            }
            clear = true;

            ++i;
//            std::cout<<i<<": "<<min_dk<<std::endl;
        }
//        std::cout<<"\n\n\n";




    }

}