#pragma once

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

class Brain; 
using namespace BT;

void RegisterStrikerDecisionNodes(BT::BehaviorTreeFactory &factory, Brain* brain);

class StrikerDecision : public SyncActionNode
{
public:
    StrikerDecision(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("chase_threshold", 1.0, "이 거리보다 멀어지면 공을 추격하는 동작을 수행"),
            InputPort<string>("decision_in", "", "이전 틱에서의 decision 값을 읽기 위해 사용"),
            InputPort<string>("position", "offense", "offense | defense, 공을 어느 방향으로 찰지 결정"),
            InputPort<double>("kick_y_offset", -0.077, "킥 시 공을 로봇 중심에서 y축으로 얼마나 오프셋 시킬지 결정"),
            InputPort<double>("set_piece_goal_dist", 2.0, "이 거리 이내에서는 정렬 생략하고 One Touch 슛 시도"),
            OutputPort<string>("decision_out")};
    }

    NodeStatus tick() override;

private:
    Brain *brain;
    double lastDeltaDir = 0.0; 
    rclcpp::Time timeLastTick; 
};