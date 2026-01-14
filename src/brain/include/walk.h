#pragma once

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

class Brain; 
using namespace BT;

/*
    해당 파일은 Walk을 위한 노드와 함수만 모아놓은 헤더 파일입니다.
*/

void RegisterWalkNodes(BT::BehaviorTreeFactory &factory, Brain* brain);

class SetVelocity : public SyncActionNode{
public:
    // 생성자 -> name : 함수 이름 / config : 노드의 설정 정보 (입/출력 포트 정보 등) / brain : 메인 Brain 클래스의 주소(포인터)
    SetVelocity(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    NodeStatus tick() override;
    static PortsList providedPorts(){ // XML 파일과 C++ 코드 사이의 연결 통로
        return {
            InputPort<double>("x", 0, "Default x is 0"),
            InputPort<double>("y", 0, "Default y is 0"),
            InputPort<double>("theta", 0, "Default  theta is 0"),
        };
    }

private:
    Brain *brain;
};


class StepOnSpot : public SyncActionNode{ // 제자리걸음
public:
    StepOnSpot(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    NodeStatus tick() override;
    static PortsList providedPorts(){
        return {};
    }

private:
    Brain *brain;
};

class GoToPosition : public SyncActionNode
{
public:
    GoToPosition(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("turn_threshold", 0.1, "turn threshold"),
            InputPort<double>("stop_threshold", 0.1, "stop threshold"),
            InputPort<double>("vx_limit", 0.5, "vx limit"),
            InputPort<double>("vy_limit", 0.5, "vy limit"),
            InputPort<double>("x", 0.0, "target x"),
            InputPort<double>("y", 0.0, "target y"),
            InputPort<double>("theta", 0.0, "target theta, degree")};
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};
