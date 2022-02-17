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

#include "fsm_bump_go/BumpGo_Advanced.h"

#include "kobuki_msgs/BumperEvent.h"

namespace fsm_bump_go
{

BumpGo_Advanced::BumpGo_Advanced()
: BaseClass::BaseClass()
{
  sub_bumper_ = n_.subscribe("/mobile_base/events/bumper", 1, &BumpGo_Advanced::bumperCallback, this);
}

void
BumpGo_Advanced::bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
  bump_ = msg->bumper;
  detected_obs_ = msg->state == kobuki_msgs::BumperEvent::PRESSED;

  front_obstacle = bump_ == kobuki_msgs::BumperEvent::CENTER;
  right_obstacle = bump_ == kobuki_msgs::BumperEvent::RIGHT;
  left_obstacle = bump_ == kobuki_msgs::BumperEvent::LEFT;
}

}  // namespace fsm_bump_go
