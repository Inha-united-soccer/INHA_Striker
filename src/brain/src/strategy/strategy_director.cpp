#include "brain.h"
#include "strategy/strategy_director.h"

PortsList StrategyDirector::providedPorts()
{
    return { OutputPort<string>("current_mode") };
}

NodeStatus StrategyDirector::tick()
{
    // 1. 블랙보드에서 게임 상태 받아오기
    bool isWinning = brain->tree->getEntry<bool>("GameState.isWinning");
    bool isLosing = brain->tree->getEntry<bool>("GameState.isLosing");
    double timeRemaining = brain->tree->getEntry<double>("GameState.timeRemaining");

    // 2. 전략 결정
    string mode = "NORMAL";

    if (isLosing && timeRemaining < 300.0) {
        mode = "ALL_OUT_ATTACK"; // 지고 있고 시간도 없다면 총공격
    } 
    else if (isWinning && timeRemaining < 300.0) {
        mode = "TIME_WASTING"; // 이기고 있고 시간이 얼마 안남았다면 시간 보내기
    } 
    else if (isWinning) {
        mode = "DEFENSIVE"; // 이기고 시간이 있다면 수비
    } 
    else {
        mode = "OFFENSIVE"; // 지고 있다면 공격
    }

    // 3. 블랙보드에 전략 넘겨주기
    setOutput("current_mode", mode);
    brain->tree->blackboard->set("Strategy.currentMode", mode);

    return NodeStatus::SUCCESS;
}
