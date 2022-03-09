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


#include "fsm_bump_go/Back.h"
#include "fsm_bump_go/Turn.h"
#include "fsm_bump_go/Is_bumped.h"
#include "fsm_bump_go/Forward.h"

#include "ros/ros.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

#include "ros/package.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "behavior_tree");
  ros::NodeHandle n;

  BT::BehaviorTreeFactory factory;//creo el arboll

  //registro los nodos los cuales añado y compilo como librerias en el cmakelists. Ademas en la clase de cada nodo al final, pones el metodo para que se 
  //identifique el nombre del nodo a su clase
  
  factory.registerNodeType<fsm_bump_go::Back>("Back");
  factory.registerNodeType<fsm_bump_go::Is_bumped>("Is_bumped");
  factory.registerNodeType<fsm_bump_go::Turn>("Turn");
  factory.registerNodeType<fsm_bump_go::Forward>("Forward");

  auto blackboard = BT::Blackboard::create();//creo la blackboard

  blackboard->set("object", "cup");

  std::string pkgpath = ros::package::getPath("fsm_bump_go");//buscas el behavior tree creado con xml
  std::string xml_file = pkgpath + "/behavior_trees_xml/tree_bump.xml";

  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);//creas el arbol
  auto publisher_zmq = std::make_shared<BT::PublisherZMQ>(tree, 10, 1666, 1667);

  ros::Rate loop_rate(5);

  int count = 0;

  bool finish = false;
  while (ros::ok() && !finish)
  {
    finish = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
