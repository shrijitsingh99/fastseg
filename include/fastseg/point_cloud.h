//
// Created by shrijitsingh99 on 10/13/19.
//

#ifndef FAST_SEG_INCLUDE_FASTSEG_POINT_CLOUD_H_
#define FAST_SEG_INCLUDE_FASTSEG_POINT_CLOUD_H_

#define PCL_NO_PRECOMPILE
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace point_cloud {
struct PointXYZIRL {
  PCL_ADD_POINT4D;
  float intensity;
  uint16_t ring;
  uint16_t label;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
}

POINT_CLOUD_REGISTER_POINT_STRUCT(point_cloud::PointXYZIRL,
                                  (float, x, x)
                                      (float, y, y)
                                      (float, z, z)
                                      (float, intensity, intensity)
                                      (uint16_t, ring, ring)
                                      (uint16_t, label, label))

namespace point_cloud {

typedef enum PointField {
  X, Y, Z, INTENSITY, RING, LABEL
} PointFields;

inline void sort(pcl::PointCloud<PointXYZIRL> &point_cloud, PointFields field) {
  auto comp = [&](PointXYZIRL a, PointXYZIRL b) -> bool {
    switch (field) {
      case X:
        return a.x < b.x;
        break;
      case Y:
        return a.y < b.y;
        break;
      case Z:
        return a.z < b.z;
        break;
      case INTENSITY:
        return a.intensity < b.intensity;
        break;
      case RING:
        return a.ring < b.ring;
        break;
      case LABEL:
        return a.label < b.label;
        break;
    }
    return false;
  };

  sort(point_cloud.points.begin(), point_cloud.points.end(), comp);
}

inline void set_attributes(pcl::PointCloud<PointXYZIRL> &point_cloud, uint32_t width, uint32_t height) {
  point_cloud.width = width;
  point_cloud.height = height;
}

inline void point_cloud_to_vector(const pcl::PointCloud<PointXYZIRL> &point_cloud, std::vector<PointXYZIRL> points_vector) {
  for(auto point = point_cloud.begin(); point < point_cloud.end(); ++point){
    points_vector.push_back(*point);
  }
}
}


#endif //FAST_SEG_INCLUDE_FASTSEG_POINT_CLOUD_H_
