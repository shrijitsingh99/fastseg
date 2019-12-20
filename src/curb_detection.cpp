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
                                                  std::vector<PointXYZIRL> &beams, std::vector<PointXYZIRL> &directions) {
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

        std::vector<point_cloud::PointXYZIRL> surround_beams;
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
            if (!clear) {
                beams.push_back(beam_min_point[i]);
                surround_beams.push_back(beam_min_point[i]);
            }
            else {
                edge_point.x = 100 * cos(i * M_PI / 180);
                edge_point.y = 100 * sin(i * M_PI / 180);
                beams.push_back(edge_point);
                surround_beams.push_back(edge_point);
            }
            clear = true;

            ++i;
        }

        auto peaks = find_peaks(surround_beams);
        for (auto point: peaks) {
            directions.push_back(sensor_origin);
            directions.push_back(point);
        }
    }

    std::vector<PointXYZIRL> CurbDetection::find_peaks(std::vector<PointXYZIRL> beams) {

        double avg_beam_length = 0.0, highest_peak_length = 0.0;
        PointXYZIRL highest_peak;
        int i = 1;
        for (auto point: beams) {
            double peak_length = pow(pow(point.x, 2) + pow(point.y, 2), 0.5);
            if (peak_length > highest_peak_length) {
                highest_peak_length = peak_length;
                highest_peak = point;
            }

            avg_beam_length += peak_length;
            ++i;
        }
        avg_beam_length /= i;

        std::vector<std::pair<PointXYZIRL, int>> maximas;

        for (int j = 0; j < beams.size() - 1; ++j) {
            if (pow(pow(beams[j].x, 2) + pow(beams[j].y, 2), 0.5) > avg_beam_length) {
                if (pow(pow(beams[j].x, 2) + pow(beams[j].y, 2), 0.5) >= pow(pow(beams[j - 1].x, 2) + pow(beams[j - 1].y, 2), 0.5)) {
                    if (pow(pow(beams[j].x, 2) + pow(beams[j].y, 2), 0.5) >= pow(pow(beams[j + 1 % beams.size()].x, 2) + pow(beams[j + 1 % beams.size()].y, 2), 0.5)) {
                        if (pow(pow(beams[j].x, 2) + pow(beams[j].y, 2), 0.5) / pow(pow(highest_peak.x, 2) + pow(highest_peak.y, 2), 0.5) >= 0.25) {
                            maximas.push_back(std::pair<PointXYZIRL, int>(beams[j], j));
                        }
                    }
                }
            }
        }

        bool merged_all = false;
        bool end_merge = false;

//        std::cout<<"Before Merge: "<<maximas.size()<<std::endl;

        sort( maximas.begin( ), maximas.end( ), [ ]( const std::pair<PointXYZIRL, int>& lhs, const std::pair<PointXYZIRL, int>& rhs )
        {
            return lhs.second < rhs.second;
        });

//        while (!merged_all) {
            merged_all = true;
            for (auto it = maximas.begin(); it < maximas.end(); ++it) {
                auto next_it = it + 1;

                if (it == maximas.end() - 1) {
                    next_it = maximas.begin();
                    end_merge = true;
                }

                int angle_diff = std::abs((*it).second - (*next_it).second);
//                std::cout<<(*it).second<<" - "<<(*next_it).second<<" = "<<angle_diff;

                angle_diff = (angle_diff > 180) ? std::abs(360 - angle_diff) : angle_diff;
                if (angle_diff <= 360 / 8 && angle_diff > 0) {
                    if ((pow(pow((*it).first.x, 2) + pow((*it).first.y, 2), 0.5)) >=
                        pow(pow((*(next_it)).first.x, 2) + pow((*(next_it)).first.y, 2), 0.5)) {
                            maximas.erase(next_it);
//                            std::cout<<" - Next"<<std::endl;
                            --it;
                            merged_all = false;
//                            continue;
                    }
                    else {
                        it = maximas.erase(it);
                        --it;
//                        std::cout<<" - Current"<<std::endl;
                        merged_all = false;
//                        continue;
                    }
                }

                if (end_merge) break;

            }
//        }


        avg_beam_length = 0.0;

        std::vector<PointXYZIRL> directions;
        for (auto it = maximas.begin(); it < maximas.end() ; ++it) {
//            int max = std::max((*(it)).second, (*(it + 1)).second);
//            int min = std::min((*(it)).second, (*(it + 1)).second);
//            double avg = 0.0;
//            int j = 1;
//            for (int i = 0; i < beams.size(); ++i) {
//                if (i >= min and i <= max) {
//                    avg += pow(pow(beams[i].x, 2) + pow(beams[i].y, 2), 0.5);
//                    ++j;
//                }
//            }
//            avg /= j;
//
//
//            double len1 = pow(pow((*it).first.x, 2) + pow((*it).first.y, 2), 0.5);
//            double len2 = pow(pow((*(it + 1)).first.x, 2) + pow((*(it + 1)).first.y, 2), 0.5);
//            if (2 * avg / (len1 + len2) < 0.8) {
                directions.push_back((*it).first);
//            }
        }


        std::cout<<"Total: "<<maximas.size()<<" | Maximas: "<<directions.size()<<std::endl;

        return directions;
    }

}