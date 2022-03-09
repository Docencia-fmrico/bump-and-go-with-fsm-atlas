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

#define PI 3.14159265

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
  int n_reading = 1;  // number of laser readings 
  
  for (int i = 1; i < msg->ranges.size(); i++)
  {
    if (msg->ranges[i] < nearest_obs_d && msg->ranges[i] > RANGE_MIN_DETECTED)
    {
      nearest_obs_d = msg->ranges[i];
      n_reading = i+1;
    }
  }


  if (nearest_obs_d < RANGE_MAX_DETECTED)
  {
    float angle_obs = msg->angle_min + msg->angle_increment*n_reading;
    
    left_obstacle_ = angle_obs > ANGLE_MIN_DETECTED && angle_obs > 0;
    right_obstacle_ = angle_obs < 0 && angle_obs < ANGLE_MAX_DETECTED;
    detected_obs_ = left_obstacle_ || right_obstacle_;
  }

  else 
  {
    detected_obs_ = false;
  }
}

}  // namespace fsm_bump_go
