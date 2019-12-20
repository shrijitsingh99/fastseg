#include <ros/ros.h>
#include "fastseg/fastseg.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "fastseg");

  fastseg::FastSeg fastseg;

  ros::spin();
}