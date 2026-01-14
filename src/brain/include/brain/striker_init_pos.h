#pragma once

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

class Brain; 
using namespace BT;

void RegisterStrikerInitPosNodes(BT::BehaviorTreeFactory &factory, Brain* brain);

class StrikerInitPos : public SyncActionNode
{
public:
    StrikerInitPos(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("turn_threshold", 0.1, "turn threshold"),
            InputPort<double>("stop_threshold", 0.1, "stop threshold"),
            InputPort<double>("vx_limit", 0.5, "vx limit"),
            InputPort<double>("vy_limit", 0.5, "vy limit"),
            InputPort<double>("init_pos_x", 0.0, "init pos x"),
            InputPort<double>("init_pos_y", 0.0, "init pos y"),
            InputPort<double>("init_pos_theta", 0.0, "init pos theta, degree")};
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};
