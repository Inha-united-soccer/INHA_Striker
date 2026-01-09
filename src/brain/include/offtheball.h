#pragma once

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

class Brain; 
using namespace BT;


void RegisterOfftheballNodes(BT::BehaviorTreeFactory &factory, Brain* brain);

class OfftheballPosition : public SyncActionNode
{
public:
    OfftheballPosition(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("dist_from_goal", 2.0, "goal 앞에서 대기할 거리"),
        };
    }

    NodeStatus tick() override;


private:
    Brain *brain;
};