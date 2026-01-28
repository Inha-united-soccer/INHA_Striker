#pragma once

#include <behaviortree_cpp/bt_factory.h>

class Brain;

void RegisterTacticsNodes(BT::BehaviorTreeFactory &factory, Brain* brain);
