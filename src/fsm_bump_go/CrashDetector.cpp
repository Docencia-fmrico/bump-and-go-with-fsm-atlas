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

#include "fsm_bump_go/CrashDetector.h"

#include "ros/ros.h"

namespace fsm_bump_go
{

CrashDetector::CrashDetector()
: detected_obs_bumper(false), detected_obs_security_area(false)
{
  sub_bumper_ = n_.subscribe("/mobile_base/events/bumper", 1, &CrashDetector::BumperCallback, this);
  sub_laser_ = n_.subscribe("/scan", 1, &CrashDetector::LaserCallback, this);
}

void CrashDetector::BumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
  bumper_msg = msg->bumper;
  detected_obs_bumper = msg->state == kobuki_msgs::BumperEvent::PRESSED;

  if(bumper_msg == kobuki_msgs::BumperEvent::CENTER)
    sector_detected_bumper = FRONT_SECTOR_BUMPER;
  else if(bumper_msg == kobuki_msgs::BumperEvent::RIGHT)
    sector_detected_bumper = RIGHT_SECTOR_BUMPER;
  else if(bumper_msg == kobuki_msgs::BumperEvent::LEFT)
    sector_detected_bumper = LEFT_SECTOR_BUMPER;

}

void CrashDetector::LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  position_array_nearest_object = 0;
  max_size_array_laser = msg->ranges.size();
  position_array_start_left = max_size_array_laser-max_size_array_laser/4;
  position_array_start_center = max_size_array_laser - max_size_array_laser/16;
  position_array_finish_center =max_size_array_laser/16;
  position_array_finish_right = max_size_array_laser/4;
  nearest_obs_d = msg->ranges[position_array_start_left]; 

  //analyze from middle-front sector to right sector
  for (int i = 0; i <= position_array_finish_right; i++)
  {
    if (msg->ranges[i] < nearest_obs_d && msg->ranges[i] > 0)
    {
      nearest_obs_d = msg->ranges[i];
      position_array_nearest_object = i;
    }
  }

  //analyze from middle-front sector to left sector
  for (int i = max_size_array_laser-1; i >= position_array_start_left; i--)
  {
    if (msg->ranges[i] < nearest_obs_d && msg->ranges[i] > 0)
    {
      nearest_obs_d = msg->ranges[i];
      position_array_nearest_object = i;
    }
  }

  //analyze the back sector
  for (int i = position_array_finish_right+1; i <= position_array_start_left; i++)
  {
    if (msg->ranges[i] < nearest_obs_d && msg->ranges[i] > 0)
    {
      nearest_obs_d = msg->ranges[i];
      position_array_nearest_object = i;
    }
  }

  if (std::isfinite(msg->ranges[position_array_nearest_object]) && msg->ranges[position_array_nearest_object] > 0)
  {
    detected_obs_security_area = msg->ranges[position_array_nearest_object] <= SECURE_DISTANCE_LASER;
  }

  if (detected_obs_security_area)
  {
    if (position_array_nearest_object > position_array_finish_center && position_array_nearest_object < position_array_finish_right) 
      sector_detected_laser = RIGHT_SECTOR_LASER;

    else if (position_array_nearest_object > position_array_start_left  && position_array_nearest_object < position_array_start_center)
      sector_detected_laser = LEFT_SECTOR_LASER;

    else if (position_array_nearest_object > position_array_start_center || position_array_nearest_object < position_array_finish_center)
      sector_detected_laser = FRONT_SECTOR_LASER;

    else
      sector_detected_laser = BACK_SECTOR_LASER;

  }
  
}


}  // namespace fsm_bump_go
