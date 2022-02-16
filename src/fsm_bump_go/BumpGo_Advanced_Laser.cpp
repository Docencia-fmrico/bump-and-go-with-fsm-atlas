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
  sub_laser_ = n_.subscribe("/scan_filtered", 700, &BumpGo_Advanced_Laser::laserCallback, this);
}

void
BumpGo_Advanced_Laser::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  front_obstacle = msg->ranges[msg->ranges.size()/2] <= 0.5;
  left_obstacle = msg->ranges[msg->ranges.size()/4] <= 0.5;
  right_obstacle = msg->ranges[msg->ranges.size()-msg->ranges.size()/4] <= 0.5;

  detected_obs_ = front_obstacle || right_obstacle || left_obstacle;
  
  ROS_INFO("%f %d %d",msg->ranges[msg->ranges.size()-msg->ranges.size()/4],detected_obs_,right_obstacle);
}

}  // namespace fsm_bump_go
