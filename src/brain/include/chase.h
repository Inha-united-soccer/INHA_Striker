#pragma once

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

class Brain; 
using namespace BT;

void RegisterChaseNodes(BT::BehaviorTreeFactory &factory, Brain* brain);

class SimpleChase : public SyncActionNode
{
public:
    SimpleChase(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("stop_dist", 1.0, "공과의 거리가 이 값보다 가까우면 더 이상 공을 향해 이동하지 않음"),
            InputPort<double>("stop_angle", 0.1, "공의 각도가 이 값 이내이면 더 이상 공을 향해 회전하지 않음"),
            InputPort<double>("vy_limit", 0.2, "보행 안정성을 위해 Y 방향 속도를 제한하며 효과를 내려면 로봇 최대 속도 0.4보다 작아야 함"),
            InputPort<double>("vx_limit", 0.6, "보행 안정성을 위해 X 방향 속도를 제한하며 효과를 내려면 로봇 최대 속도 1.2보다 작아야 함"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};

class Chase : public SyncActionNode
{
public:
    Chase(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("vx_limit", 0.6, "追球的最大 x 速度"),
            InputPort<double>("vy_limit", 0.4, "追球的最大 y 速度"),
            InputPort<double>("vtheta_limit", 1.0, "追球时, 实时调整方向的速度不大于这个值"),
            InputPort<double>("dist", 0.1, "追球的目标是球后面多少距离"),
            InputPort<double>("safe_dist", 4.0, "circle back 时, 保持的安全距离"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
    string _state;     
    double _dir = 1.0; 
};

class DribbleChase : public SyncActionNode
{
public:
    DribbleChase(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("min_speed", 0.2, "Minimum approach speed"),
            InputPort<double>("max_speed", 0.8, "Maximum approach speed (dribble run)"),
            InputPort<double>("slow_dist_far", 0.8, "Distance to start speeding up"),
            InputPort<double>("slow_dist_near", 0.3, "Distance to start slowing down"),
            InputPort<double>("vx_limit", 0.6, "Max X speed limit"),
            InputPort<double>("vy_limit", 0.4, "Max Y speed limit"),
            InputPort<double>("vtheta_limit", 1.0, "Max Turn speed limit"),
        };
    }

    NodeStatus tick() override;
    
private:
    Brain *brain;
};

class DribbleToGoal : public BT::StatefulActionNode
{
public:
    DribbleToGoal(const string &name, const BT::NodeConfig &config, Brain *brain)
        : BT::StatefulActionNode(name, config), brain(brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("vx_limit", 1.0, "x velocity limit"),
            BT::InputPort<double>("vy_limit", 0.5, "y velocity limit"),
            BT::InputPort<double>("vtheta_limit", 1.0, "theta velocity limit"),
            BT::InputPort<double>("dist_to_goal", 2.0, "stop distance from goal"),
            // Dribble Speed Params
            BT::InputPort<double>("min_speed", 0.3, "Minimum approach speed"),
            BT::InputPort<double>("max_speed", 0.8, "Maximum approach speed"),
            BT::InputPort<double>("slow_dist_far", 0.8, "Distance to start speeding up"),
            BT::InputPort<double>("slow_dist_near", 0.4, "Distance to start slowing down"),
        };
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    Brain *brain;
};

class OfftheballPosition : public StatefulActionNode
{
public:
    OfftheballPosition(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

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