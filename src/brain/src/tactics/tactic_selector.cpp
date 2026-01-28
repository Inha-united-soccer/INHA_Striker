#include "brain.h"
#include "tactics/tactic_selector.h"

PortsList TacticSelector::providedPorts()
{
    return { OutputPort<string>("active_tactic") };
}

NodeStatus TacticSelector::tick()
{
    // 1. 블랙보드에서 현재 전략 읽어오기
    string strategyMode = brain->tree->getEntry<string>("Strategy.currentMode");

    // 2. 전략에 따른 tactics 선택
    string selectedTactic = "LINE_DEFENSE"; // Default -> 상대 전력에 대한 결정을 default로 설정해준다?

    if (strategyMode == "ALL_OUT_ATTACK") {
        selectedTactic = "PRESSING";
    } 
    else if (strategyMode == "OFFENSIVE") {
        selectedTactic = "PRESSING";
    } 
    else if (strategyMode == "DEFENSIVE") {
        selectedTactic = "LINE_DEFENSE";
    } 
    else if (strategyMode == "TIME_WASTING") {
        selectedTactic = "TEMPO_CONTROL";
    }

    // 3. 블랙보드에 선택된 테크닉 넘겨주기
    setOutput("active_tactic", selectedTactic);
    brain->tree->blackboard->set("Tactics.activeTactic", selectedTactic);

    return NodeStatus::SUCCESS;
}
