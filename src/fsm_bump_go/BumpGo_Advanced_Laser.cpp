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

#include "fsm_bump_go/BumpGo_Advanced_Laser.h"

#include "sensor_msgs/LaserScan.h"

namespace fsm_bump_go
{

BumpGo_Advanced_Laser::BumpGo_Advanced_Laser()
: BaseClass::BaseClass()
{
  sub_laser_ = n_.subscribe("/scan_filtered", 1, &BumpGo_Advanced_Laser::laserCallback, this);
}

void
BumpGo_Advanced_Laser::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  float nearest_obs_d = msg->ranges[0]; 
  int index = 0;
  for (int i = 1; i < msg->ranges.size()-1; i++)
  {
    if (msg->ranges[i] < nearest_obs_d && msg->ranges[i] > RANGE_MIN_DETECTED)
    {
      nearest_obs_d = msg->ranges[i];
      index = i;
    }
  }

  detected_obs_ = nearest_obs_d > RANGE_MIN_DETECTED && nearest_obs_d < RANGE_MAX_DETECTED;
  
  if (detected_obs_)
  {
    float angle_detected_obs_ = msg->angle_min + msg->angle_increment*index;

    left_obstacle_ = angle_detected_obs_ < msg->angle_min/2 && angle_detected_obs_ > 0;
    right_obstacle_ = angle_detected_obs_ < 0 && angle_detected_obs_ > msg->angle_max/2;
  }

  //ROS_INFO("%d %d", left_obstacle_, right_obstacle_);
}

}  // namespace fsm_bump_go
