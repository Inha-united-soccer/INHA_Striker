#pragma once

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

class Brain;
using namespace BT;
using namespace std;

class GameStateManager : public SyncActionNode
{
public:
    GameStateManager(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts();
    NodeStatus tick() override;

private:
    Brain *brain;
};
