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
  left_obstacle = false;
  right_obstacle = false;
  front_obstacle = false;

  const int range_max= msg->ranges.size()/4;
  const int range_min = msg->ranges.size()-msg->ranges.size()/4;
  const int range_front_min = msg->ranges.size() - msg->ranges.size()/12;
  const int range_front_max = msg->ranges.size()/12;



  float nearest_obs_d = msg->ranges[range_min]; 
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
    detected_obs_ = msg->ranges[n] <= 0.4;

  }

  if (detected_obs_)
  {
  
    if (n > range_front_max && n < range_max) 
    {
      right_obstacle = true;
      ROS_INFO("Derecha %d", right_obstacle);

    }
    else if (n > range_min  && n < range_front_min)
    {
      left_obstacle = true;
      ROS_INFO("Izquierda %d", left_obstacle);
      
    }
    else if (n > range_front_min || n < range_front_max)
    {
      front_obstacle = true; 
      ROS_INFO("Delante %d", front_obstacle);   
    }
  }
  
  ROS_INFO(" %ld %d %d %d %d %d",msg->ranges.size(),n,left_obstacle,right_obstacle,front_obstacle, detected_obs_);
}

}  // namespace fsm_bump_go
