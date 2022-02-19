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
  const int RANGE_MAX = msg->ranges.size()/4;
  const int RANGE_MIN = msg->ranges.size()-msg->ranges.size()/4;
  const int RANGE_FRONT_MIN = msg->ranges.size() - msg->ranges.size()/16;
  const int RANGE_FRONT_MAX = msg->ranges.size()/16;


  float nearest_obs_d = msg->ranges[RANGE_MIN]; 
  int n = 0;
  for (int i = 0; i <= RANGE_MAX; i++)
  {
    if (msg->ranges[i] < nearest_obs_d && msg->ranges[i] > 0)
    {
      nearest_obs_d = msg->ranges[i];
      n = i;
    }
  }

  for (int i = msg->ranges.size()-1; i >= RANGE_MIN; i--)
  {
    if (msg->ranges[i] < nearest_obs_d && msg->ranges[i] > 0)
    {
      nearest_obs_d = msg->ranges[i];
      n = i;
    }
  }

  if (std::isfinite(msg->ranges[n]) && msg->ranges[n] > 0)
  {
    detected_obs_ = msg->ranges[n] <= DETECTED_OBS_DISTANCE;
  }

  if (detected_obs_)
  {
    if (n > RANGE_FRONT_MAX && n < RANGE_MAX) 
    {
      right_obstacle = true;
      left_obstacle = false;
      front_obstacle = false;
    
    }
    else if (n > RANGE_MIN  && n < RANGE_FRONT_MIN)
    {
      left_obstacle = true;
      right_obstacle = false;
      front_obstacle = false;
      
    }
    else if (n > RANGE_FRONT_MIN || n < RANGE_FRONT_MAX)
    {
      front_obstacle = true; 
      left_obstacle = false;
      right_obstacle = false;
      
    }
  }
  
}

}  // namespace fsm_bump_go
