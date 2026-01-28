#include "brain.h"
#include "strategy/game_state_manager.h"

PortsList GameStateManager::providedPorts()
{
    return { OutputPort<bool>("is_winning"),
             OutputPort<bool>("is_losing"),
             OutputPort<double>("time_remaining") };
}

NodeStatus GameStateManager::tick()
{
    
    int myScore = brain->data->gameControlData.score[0]; // 0: My team
    int oppScore = brain->data->gameControlData.score[1]; 
    int timeRemaining = brain->data->gameControlData.secsRemaining;

    bool isWinning = myScore > oppScore;
    bool isLosing = myScore < oppScore;

    setOutput("is_winning", isWinning);
    setOutput("is_losing", isLosing);
    setOutput("time_remaining", (double)timeRemaining);

    brain->tree->setEntry("GameState.isWinning", isWinning);
    brain->tree->setEntry("GameState.isLosing", isLosing);
    brain->tree->setEntry("GameState.timeRemaining", (double)timeRemaining);
    
    return NodeStatus::SUCCESS;
}
