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
    string selectedTactic = "NORMAL"; // Default Tactics 모드 

    // 3. 변수 선언
    int myScore = brain->data->gameControlData.score[0];
    int oppScore = brain->data->gameControlData.score[1];
    int scoreDiff = myScore - oppScore;
    double ballX = brain->data->ball.posToField.x;

    // =============================== [총공격 Strategy일 때] ===============================
    if (strategyMode == "ALL_OUT_ATTACK") { 
        selectedTactic = "TOTAL_ASSAULT"; // striker는 파라미터 키우는 데에 그치지만 goalkeeper라면 공격에 합류할 정도로 급박한 상황으로 설정
    } 
    // =============================== [공격 Strategy일 때] ===============================
    else if (strategyMode == "OFFENSIVE") { 
        if (scoreDiff <= -3) {
            selectedTactic = "TOTAL_ASSAULT"; // 1. 크게 지고 있으면: 총공격
        } 
        else if (ballX < -1.0) {
            selectedTactic = "COUNTER_ATTACK"; // 2. 우리 진영에서 공을 잡으면: 빠른 역습
        }
        else {
            selectedTactic = "PRESSING"; // 3. 그 외(상대 진영): 전방 압박
        }
    } 
    // =============================== [수비 Strategy일 때] ===============================
    else if (strategyMode == "DEFENSIVE") { 
        if (scoreDiff >= 3) {
            selectedTactic = "TEMPO_CONTROL"; // 1. 승리 확실: 템포 조절
        } 
        else if (ballX < -3.0) {
            selectedTactic = "DEEP_DEFENSE"; // 2. 실점 위기: 버스 세우기 수비
        }
        else {
            selectedTactic = "LINE_DEFENSE"; // 3. 일반적인 수비: 라인 유지
        }
    } 
    // =============================== [시간 보내기 Strategy일 때] ===============================
    else if (strategyMode == "TIME_WASTING") {
        if (ballX < -3.0) {
            selectedTactic = "DEEP_DEFENSE"; // 1. 시간 끌다가도 위험하면 텐백
        } 
        else {
            selectedTactic = "TEMPO_CONTROL"; // 2. 그 외에는 시간 보내기
        }
    }

    // 3. 블랙보드에 선택된 테크닉 넘겨주기
    setOutput("active_tactic", selectedTactic);
    brain->tree->setEntry("Tactics.activeTactic", selectedTactic);

    return NodeStatus::SUCCESS;
}
