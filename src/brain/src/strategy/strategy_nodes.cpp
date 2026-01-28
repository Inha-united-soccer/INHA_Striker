#include "brain.h"
#include "strategy/strategy_nodes.h"
#include "strategy/game_state_manager.h"
#include "strategy/strategy_director.h"

using namespace std;
using namespace BT;

#define REGISTER_STRATEGY_BUILDER(Name)     \
    factory.registerBuilder<Name>( \
        #Name,                     \
        [brain](const string &name, const NodeConfig &config) { return make_unique<Name>(name, config, brain); });

void RegisterStrategyNodes(BT::BehaviorTreeFactory &factory, Brain* brain){
    REGISTER_STRATEGY_BUILDER(GameStateManager);
    REGISTER_STRATEGY_BUILDER(StrategyDirector);
}
