li_slam_ros2
====
![foxy](https://github.com/rsasaki0109/li_slam_ros2/workflows/foxy/badge.svg)  



This package is a combination of [lidarslam_ros2](https://github.com/rsasaki0109/lidarslam_ros2) and the [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) IMU composite method.

See LIO-SAM for IMU composites, otherwise see lidarslam_ros2.

<img src="./scanmatcher/images/li_slam.png">

Green path: path

Reference(From the LIO-SAM paper)  
https://github.com/TixiaoShan/LIO-SAM/blob/master/config/doc/paper.pdf  
<img src="./scanmatcher/images/liosam_thesis.png">

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
git clone https://github.com/borglab/gtsam
cd gtsam
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

The optimization pipeline in Lidar Inertial SLAM were taken from [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM).

(Note: See the LIO-SAM repository for detailed settings regarding IMU.  
The other thing to note is that the speed will diverge if the voxel_grid_size is large.  

demo data(ROS1) in LIO-SAM   
https://github.com/TixiaoShan/LIO-SAM   
The Velodyne VLP-16 was used in this data.


```
rviz2 -d src/li_slam_ros2/scanmatcher/rviz/lio.rviz 
```

```
ros2 launch scanmatcher lio.launch.py
```

```
ros2 bag play -s rosbag_v2 casual_walk.bag 
```

<img src="./scanmatcher/images/li_slam.gif">

Green arrow: pose, Yellow path: path, Green path: path by imu 

<img src="./scanmatcher/images/li_slam.png">

Green path: path

rosgraph

<img src="./scanmatcher/images/rosgraph.png">  

`pose_graph.g2o` and `map.pcd` are saved in loop closing or using the following service call.

```
ros2 service call /map_save std_srvs/Empty
```


## Used Libraries 

- Eigen
- PCL(BSD3)
- g2o(BSD2 except a part)
- [ndt_omp](https://github.com/koide3/ndt_omp) (BSD2)
- gtsam(BSD2)
