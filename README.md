# indy_pcl

The src/indy_pcl.cpp file will subscribe to the topic /tf & /carla/vehicle/086/lidar/front/point_cloud, will use two call back functions to receive data from these two topics. The cpp file will use /tf topic messages to transform point clouds and merge point clouds in a single point cloud. After ros shutdown the single point cloud will convert into Merged.pcd file.
