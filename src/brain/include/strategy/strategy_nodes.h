#pragma once

#include <behaviortree_cpp/bt_factory.h>

class Brain;

void RegisterStrategyNodes(BT::BehaviorTreeFactory &factory, Brain* brain);
