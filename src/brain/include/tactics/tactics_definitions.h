#pragma once

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

class Brain;
using namespace BT;
using namespace std;

// 전략 1. 압박 전략
class TacticPressing : public CoroActionNode
{
public:
    TacticPressing(const string &name, const NodeConfig &config, Brain *_brain) : CoroActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts();
    NodeStatus tick() override;

private:
    Brain *brain;
};

// 전략 2. 수비라인 메이킹
class TacticLineDefense : public SyncActionNode
{
public:
    TacticLineDefense(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts();
    NodeStatus tick() override;

private:
    Brain *brain;
    // State variables for smooth head movement
    rclcpp::Time scanStartTime = rclcpp::Time(0, 0, RCL_ROS_TIME);
    double smoothHeadYaw = 0.0;
};

// 전략 3. 템포 컨트롤
class TacticTempoControl : public CoroActionNode
{
public:
    TacticTempoControl(const string &name, const NodeConfig &config, Brain *_brain) : CoroActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts();
    NodeStatus tick() override;

private:
    Brain *brain;
};
