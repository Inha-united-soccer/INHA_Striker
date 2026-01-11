#pragma once

#include <string>
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"

using std::string;

class Brain;

class PassReceive : public BT::StatefulActionNode
{
public:
    PassReceive(const string &name, const BT::NodeConfig &config, Brain *brain)
        : BT::StatefulActionNode(name, config), brain(brain)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {
            // BT::InputPort<string>("_while"), // Underscore reserved error
        };
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    Brain *brain;
};

void RegisterPassReceiveNodes(BT::BehaviorTreeFactory &factory, Brain* brain);
