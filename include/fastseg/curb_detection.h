//
// Created by shrijitsingh99 on 12/17/19.
//

#ifndef FASTSEG_INCLUDE_FASTSEG_CURB_DETECTION_H_
#define FASTSEG_INCLUDE_FASTSEG_CURB_DETECTION_H_

#include <pcl/common/centroid.h>
#include "fastseg/point_cloud.h"

namespace fastseg {

using namespace point_cloud;

class CurbDetection {
 public:
    CurbDetection();
    ~CurbDetection() = default;
    void sliding_beam_segmentation(pcl::PointCloud<PointXYZIRL> &point_cloud, pcl::PointCloud<PointXYZIRL> &ground_cloud, pcl::PointCloud<PointXYZIRL> &no_ground_cloud, std::vector<PointXYZIRL> &beams);

 private:
    double max_range = 30.0;
    unsigned int num_zones = 360;
};
}

#endif //FASTSEG_INCLUDE_FASTSEG_CURB_DETECTION_H_
