
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

#ifndef FSM_BUMP_GO_BUMPGO_H
#define FSM_BUMP_GO_BUMPGO_H

#include "ros/ros.h"

#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

namespace fsm_bump_go
{

class BumpGo_Advanced_Laser
{
public:
  BumpGo_Advanced_Laser();

  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void step();

protected:
  ros::NodeHandle n_;

  static const int GOING_FORWARD   = 0;
  static const int GOING_BACK = 1;
  static const int TURNING_LEFT = 2;
  static const int TURNING_RIGHT = 3;

  static constexpr float GOING_FORWARD_VEL = 0.2;
  static constexpr float TURNING_VEL = 0.5;
  static constexpr float GOING_BACK_VEL = -0.2;

  static constexpr double TURNING_TIME = 5.0;
  static constexpr double BACKING_TIME = 3.0;

  int state_;

  bool detected_obs_;

  ros::Time detected_obs_ts_;
  ros::Time turn_ts_;

  ros::Subscriber sub_laser_;
  ros::Publisher pub_vel_;
};

}  // namespace fsm_bump_go

#endif  // FSM_BUMP_GO_BUMPGO_Hprivate