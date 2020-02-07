# fastseg

<p align="center"><img src="https://i.imgur.com/jeZnof6.gif" /></p>


# Getting Started
Clone the repository into your catkin workspace:
```sh
git clone https://github.com/shrijitsingh99/fastseg
```
Launch the node and rviz:
```sh
roslaunch fastseg fastseg.launch
```

| Subscribed Topics | Message Type            | Description           |
|-------------------|-------------------------|-----------------------|
| /cloud_in         | sensor_msgs/PointCloud2 | LiDAR pointcloud data |

| Published Topics | Message Type              | Description                                            |
|------------------|---------------------------|--------------------------------------------------------|
| /all_points      | sensor_msgs/PointCloud2   | LiDAR input point clouds                               |
| /ground          | sensor_msgs/PointCloud2   | Filtered ground points                                 |
| /no_ground       | sensor_msgs/PointCloud2   | Filtered non ground points                             |
| /beams           | visualization_msgs/Marker | Visualization of LiDAR beams as well as road direction |


# Progress
- [x] Ground plane segmentation
- [ ] Multi-plane fitting
- [x] Bottom layer beam model
- [ ] Top layer beam model
- [ ] Curb detection
- [ ] Road segmentation

# References
```bib
@inproceedings{Zermas2017Fast,
  title={Fast segmentation of 3D point clouds: A paradigm on LiDAR data for autonomous vehicle applications},
  author={Zermas, Dimitris and Izzat, Izzat and Papanikolopoulos, Nikolaos},
  booktitle={IEEE International Conference on Robotics and Automation},
  year={2017},
}

@article{Zhang2018RoadSeg,
  title={Road-Segmentation-Based Curb Detection Method for Self-Driving via a 3D-LiDAR Sensor},
  author={Yihuan Zhang, Jun Wang, Xiaonian Wang and John M. Dolan},
  journal={IEEE Transactions on Intelligent Transportation Systems (Volume: 19, Issue: 12, Dec. 2018)},
  year={2018},
}

```
