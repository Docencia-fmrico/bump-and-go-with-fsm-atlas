
// Copyright 2019 Intelligent Robotics Lab
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

#ifndef FSM_BUMP_GO_BACK_H
#define FSM_BUMP_GO_BACK_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <string>
#include "ros/ros.h"

namespace fsm_bump_go
{

class Back : public BT::ActionNodeBase
{
  public:
    ros::NodeHandle n_;
    explicit Back(const std::string& name, const BT::NodeConfiguration& config);

    void halt();

    BT::NodeStatus tick();

    static BT::PortsList providedPorts()
    {
        return { 
          BT::InputPort<ros::Time>("object1"),
          BT::OutputPort<ros::Time>("object2"),
          
        };
    }

  private:
    static constexpr float GOING_BACK_VEL = -0.2;
    static constexpr double BACKING_TIME = 3.0;
    ros::Time turn_ts_;
    ros::Publisher pub_vel_;

};

}  // namespace fsm_bump_go

#endif  // FSM_BUMP_GO_BACK_H