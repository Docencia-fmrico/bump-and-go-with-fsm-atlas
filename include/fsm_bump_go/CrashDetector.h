// Copyright 2022 Intelligent Robotics Lab
//
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

#ifndef FSM_BUMP_GO_BASECLASS_H
#define FSM_BUMP_GO_BASECLASS_H

#include "ros/ros.h"
#include "kobuki_msgs/BumperEvent.h"
#include "sensor_msgs/LaserScan.h"

namespace fsm_bump_go
{

typedef enum {LEFT_SECTOR_LASER, FRONT_SECTOR_LASER, RIGHT_SECTOR_LASER, BACK_SECTOR_LASER} laser_sector;
typedef enum {LEFT_SECTOR_BUMPER, FRONT_SECTOR_BUMPER, RIGHT_SECTOR_BUMPER} bamper_sector;

class CrashDetector
{
public:
  CrashDetector();
  void BumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg);
  void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

  bool detected_obs_bumper;
  bamper_sector sector_detected_bumper;

  bool detected_obs_security_area;
  laser_sector sector_detected_laser;
  float nearest_obs_d;
  int position_array_nearest_object;

private:
  ros::NodeHandle n_;

  const float SECURE_DISTANCE_LASER = 0.4;

  int bumper_msg;

  int max_size_array_laser;
  int position_array_finish_right;  
  int position_array_start_left;
  int position_array_start_center;
  int position_array_finish_center;

  ros::Subscriber sub_bumper_;
  ros::Subscriber sub_laser_;

};

}  // namespace fsm_bump_go

#endif  // FSM_BUMP_GO_BASECLASS_H
