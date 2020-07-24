// BSD 3-Clause License
//
// Copyright (c) 2020, Tixiao Shan
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#pragma once
#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_

// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
//#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

// #include <opencv/cv.h>
#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

using namespace std;

template < typename T >
double ROS_TIME(T msg)
{
  return msg->header.stamp.sec +
         msg->header.stamp.nanosec * 1e-9;
}

class ParamServer: public rclcpp::Node
{

public:
  string pointCloudTopic;
  string imuTopic;
  string odomTopic;

  // Velodyne Sensor Configuration: Velodyne
  int N_SCAN;
  int Horizon_SCAN;

  // IMU
  float imuAccNoise;
  float imuGyrNoise;
  float imuAccBiasN;
  float imuGyrBiasN;
  float imuGravity;
  vector < double > extRotV;
  vector < double > extRPYV;
  vector < double > extTransV;
  Eigen::Matrix3d extRot;
  Eigen::Matrix3d extRPY;
  Eigen::Vector3d extTrans;
  Eigen::Quaterniond extQRPY;

  ParamServer(std::string node_name, const rclcpp::NodeOptions & options)
    : Node(node_name, options)
  {
    declare_parameter("pointCloudTopic", "points_raw");
    get_parameter("pointCloudTopic", pointCloudTopic);
    declare_parameter("imuTopic", "imu_correct");
    get_parameter("imuTopic", imuTopic);
    declare_parameter("odomTopic", "preintegrated_odom");
    get_parameter("odomTopic", odomTopic);

    declare_parameter("N_SCAN", 16);
    get_parameter("N_SCAN", N_SCAN);
    declare_parameter("Horizon_SCAN", 1800);
    get_parameter("Horizon_SCAN", Horizon_SCAN);

    declare_parameter("imuAccNoise", 3.9939570888238808e-03);
    get_parameter("imuAccNoise", imuAccNoise);
    declare_parameter("imuGyrNoise", 1.5636343949698187e-03);
    get_parameter("imuGyrNoise", imuGyrNoise);
    declare_parameter("imuAccBiasN", 6.4356659353532566e-05);
    get_parameter("imuAccBiasN", imuAccBiasN);
    declare_parameter("imuGyrBiasN", 3.5640318696367613e-05);
    get_parameter("imuGyrBiasN", imuGyrBiasN);
    declare_parameter("imuGravity", 9.80511);
    get_parameter("imuGravity", imuGravity);
    double org_data1[] = {-1, 0, 0,
      0, 1, 0,
      0, 0, -1};
    std::vector < double > data1(org_data1, std::end(org_data1));
    declare_parameter("extrinsicRot", data1);
    get_parameter("extrinsicRot", extRotV);
    double org_data2[] = {0, 1, 0,
      -1, 0, 0,
      0, 0, 1};
    std::vector < double > data2(org_data2, std::end(org_data2));
    declare_parameter("extrinsicRPY", data2);
    get_parameter("extrinsicRPY", extRPYV);
    double org_data3[] = {0, 0, 0};
    std::vector < double > data3(org_data3, std::end(org_data3));
    declare_parameter("extrinsicTrans", data3);
    get_parameter("extrinsicTrans", extTransV);
    extRot = Eigen::Map < const Eigen::Matrix < double, -1, -1,
    Eigen::RowMajor >> (extRotV.data(), 3, 3);
    extRPY = Eigen::Map < const Eigen::Matrix < double, -1, -1,
    Eigen::RowMajor >> (extRPYV.data(), 3, 3);
    extTrans = Eigen::Map < const Eigen::Matrix < double, -1, -1,
    Eigen::RowMajor >> (extTransV.data(), 3, 1);
    extQRPY = Eigen::Quaterniond(extRPY);

  }

  sensor_msgs::msg::Imu imuConverter(const sensor_msgs::msg::Imu & imu_in)
  {
    sensor_msgs::msg::Imu imu_out = imu_in;
    // rotate acceleration
    Eigen::Vector3d acc(
      imu_in.linear_acceleration.x, imu_in.linear_acceleration.y,
      imu_in.linear_acceleration.z);
    acc = extRot * acc;
    imu_out.linear_acceleration.x = acc.x();
    imu_out.linear_acceleration.y = acc.y();
    imu_out.linear_acceleration.z = acc.z();
    // rotate gyroscope
    Eigen::Vector3d gyr(
      imu_in.angular_velocity.x, imu_in.angular_velocity.y,
      imu_in.angular_velocity.z);
    gyr = extRot * gyr;
    imu_out.angular_velocity.x = gyr.x();
    imu_out.angular_velocity.y = gyr.y();
    imu_out.angular_velocity.z = gyr.z();
    // rotate roll pitch yaw
    Eigen::Quaterniond q_from(
      imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y,
      imu_in.orientation.z);
    Eigen::Quaterniond q_final = q_from * extQRPY;
    imu_out.orientation.x = q_final.x();
    imu_out.orientation.y = q_final.y();
    imu_out.orientation.z = q_final.z();
    imu_out.orientation.w = q_final.w();

        #if 0
    cout << "extRot: " << endl;
    cout << extRot << endl;
    cout << "extRPY: " << endl;
    cout << extRPY << endl;
        #endif

    return imu_out;
  }
};


#endif
