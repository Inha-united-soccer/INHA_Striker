#include "brain.h"
#include "tactics/tactics_nodes.h"
#include "tactics/tactics_definitions.h"
#include "tactics/tactic_selector.h"

using namespace std;
using namespace BT;

#define REGISTER_TACTICS_BUILDER(Name)     \
    factory.registerBuilder<Name>( \
        #Name,                     \
        [brain](const string &name, const NodeConfig &config) { return make_unique<Name>(name, config, brain); });

void RegisterTacticsNodes(BT::BehaviorTreeFactory &factory, Brain* brain){
    REGISTER_TACTICS_BUILDER(TacticSelector);
    REGISTER_TACTICS_BUILDER(TacticPressing);
    REGISTER_TACTICS_BUILDER(TacticLineDefense);
    REGISTER_TACTICS_BUILDER(TacticTempoControl);
    REGISTER_TACTICS_BUILDER(TacticAllOut);
    REGISTER_TACTICS_BUILDER(TacticCounterAttack);
    REGISTER_TACTICS_BUILDER(TacticDeepDefense);
}
