li_slam_ros2
====
![foxy](https://github.com/rsasaki0109/li_slam_ros2/workflows/foxy/badge.svg)  

This package is a combination of [lidarslam_ros2](https://github.com/rsasaki0109/lidarslam_ros2) and the LIO-SAM IMU composite method.

See LIO-SAM for IMU composites, otherwise see lidarslam_ros2.

## requirement to build
You need  [ndt_omp_ros2](https://github.com/rsasaki0109/ndt_omp_ros2) and gtsam for scan-matcher

clone
```
cd ~/ros2_ws/src
git clone --recursive https://github.com/rsasaki0109/lidarslam_ros2
```
gtsam install
```
sudo apt-get install libtbb-dev
mkdir ~/workspace && cd ~/workspace
git clone https://github.com/rsasaki0109/gtsam-4.0.2
cd gtsam-4.0.2
mkdir -p build && cd build
cmake \
  -DCMAKE_BUILD_TYPE=Release \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
  -DGTSAM_USE_SYSTEM_EIGEN=ON \
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  ..
make -j4 check
sudo make install -j4
```
build
```
cd ~/ros2_ws
colcon build
```


## Demo

The initialization methods and the optimization pipeline in Lidar Inertial SLAM were heavily derived or taken from [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM).

(Note: See the LIO-SAM repository for detailed settings regarding IMU.  
The other thing to note is that the speed will diverge if the voxel_grid_size is large.  

demo data(ROS1) in LIO-SAM   
https://github.com/TixiaoShan/LIO-SAM   
The Velodyne VLP-16 was used in this data.


```
rviz2 -d src/lidarslam_ros2/scanmatcher/rviz/lio.rviz 
```

```
ros2 launch scanmatcher lio.launch.py
```

```
ros2 bag play -s rosbag_v2 casual_walk.bag 
```

<img src="./scanmatcher/images/li_slam.gif">

Yellow path:path, Green path: path by imu 


<img src="./scanmatcher/images/rosgraph.png">  

rosgraph


## Used Libraries 

- Eigen
- PCL(BSD3)
- g2o(BSD2 except a part)
- [ndt_omp](https://github.com/koide3/ndt_omp) (BSD2)
- gtsam
