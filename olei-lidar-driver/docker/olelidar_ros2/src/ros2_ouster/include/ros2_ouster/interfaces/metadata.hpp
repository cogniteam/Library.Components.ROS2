// Copyright 2020, Steve Macenski
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROS2_OUSTER__INTERFACES__METADATA_HPP_
#define ROS2_OUSTER__INTERFACES__METADATA_HPP_

#include <vector>
#include <string>

namespace ros2_ouster
{

/**
 * @brief client response on current state
 */
enum ClientState
{
  TIMEOUT = 0,
  ERROR = 1,
  LIDAR_DATA = 2,
  IMU_DATA = 4,
  EXIT = 8
};

/**
 * @brief metadata about Ouster lidar sensor
 */
struct Metadata
{
  std::string computer_ip;
  std::string lidar_ip;
  int imu_port;
  int lidar_port;

  std::vector<double> imu_to_sensor_transform;
  std::vector<double> lidar_to_sensor_transform;

  // added by zyl
  std::vector<double> x_offset_array;
  std::vector<double> y_offset_array;
  std::vector<double> ah_offset_array;
  std::vector<double> av_offset_array;
  std::vector<int64_t> laser_id_array;
  int num_lasers;
  double distance_resolution;
  int ring_scan;
  std::string lidar_vendor;
  int lidar_packet_size;
  int imu_packet_size;
};

}  // namespace ros2_ouster

#endif  // ROS2_OUSTER__INTERFACES__METADATA_HPP_
