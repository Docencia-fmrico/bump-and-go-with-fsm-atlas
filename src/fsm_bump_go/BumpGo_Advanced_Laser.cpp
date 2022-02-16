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
  sub_laser_ = n_.subscribe("/scan_filtered", 100, &BumpGo_Advanced_Laser::laserCallback, this);
}

void
BumpGo_Advanced_Laser::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  float nearest_obs_d = msg->ranges[0]; 
  int n = 0;
  for (int i = 0; i < msg->ranges.size(); i++)
  {
    if (msg->ranges[i] < nearest_obs_d && msg->ranges[i] > 0)
    {
      nearest_obs_d = msg->ranges[i];
      n = i;
    }
  }

  if (std::isfinite(msg->ranges[n]) && msg->ranges[n] > 0)
  {
    detected_obs_ = msg->ranges[n] <= 0.5;
  }

  if (detected_obs_)
  {
    if (n > 200 && n < msg->ranges.size()/2-250) 
    {
      left_obstacle = true;
    }
    else if (n < msg->ranges.size()-201 && n > msg->ranges.size()/2+250)
    {
      right_obstacle = true;
    }
    else 
    {
      front_obstacle = true;
    }
  }
  
  ROS_INFO("%d %d %d",left_obstacle,right_obstacle,detected_obs_);
}

}  // namespace fsm_bump_go
