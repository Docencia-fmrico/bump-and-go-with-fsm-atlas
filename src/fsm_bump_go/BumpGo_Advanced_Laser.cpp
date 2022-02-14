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
#include "geometry_msgs/Twist.h"

#include "ros/ros.h"

namespace fsm_bump_go
{

BumpGo_Advanced_Laser::BumpGo_Advanced_Laser()
: state_(GOING_FORWARD),
  detected_obs_(false)
{
  sub_laser_ = n_.subscribe("/scan_filtered", 1, &BumpGo_Advanced_Laser::laserCallback, this);
  pub_vel_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);
}

void
BumpGo_Advanced_Laser::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  if (std::isfinite(msg->ranges[msg->ranges.size()/2]))
  {
    detected_obs_ = msg->ranges[msg->ranges.size()/2] <= 0.5;
  }
  ROS_INFO("%f %d",msg->ranges[msg->ranges.size()/2],detected_obs_);
}

void
BumpGo_Advanced_Laser::step()
{
  geometry_msgs::Twist cmd;

  switch (state_)
  {
    case GOING_FORWARD:
      cmd.linear.x = GOING_FORWARD_VEL;

      if (detected_obs_)
      {
        detected_obs_ts_ = ros::Time::now();
        state_ = GOING_BACK;
        ROS_INFO("GOING_FORWARD -> GOING_BACK");
      }

      break;
    case GOING_BACK:
      cmd.linear.x = GOING_BACK_VEL;

      if ((ros::Time::now() - detected_obs_ts_).toSec() > BACKING_TIME )
      {
        turn_ts_ = ros::Time::now();
        state_ = TURNING_RIGHT;
        ROS_INFO("GOING_BACK -> TURNING");
      }

      break;
    case TURNING_RIGHT:
      cmd.angular.z = -TURNING_VEL;

      if ((ros::Time::now()-turn_ts_).toSec() > TURNING_TIME )
      {
        state_ = GOING_FORWARD;
        ROS_INFO("TURNING_RIGHT -> GOING_FORWARD");
      }
      break;
    }

    pub_vel_.publish(cmd);
}

}  // namespace fsm_bump_go