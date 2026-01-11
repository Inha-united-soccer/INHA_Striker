#include "brain.h"
#include "brain_tree.h"
#include "striker_decision.h"

#include <cstdlib>
#include <ctime>

#define REGISTER_STRIKERDECISION_BUILDER(Name)     \
    factory.registerBuilder<Name>( \
        #Name,                     \
        [brain](const string &name, const NodeConfig &config) { return make_unique<Name>(name, config, brain); });


void RegisterStrikerDecisionNodes(BT::BehaviorTreeFactory &factory, Brain* brain){
    REGISTER_STRIKERDECISION_BUILDER(StrikerDecision)
}

NodeStatus StrikerDecision::tick() {


    double chaseRangeThreshold;
    getInput("chase_threshold", chaseRangeThreshold);
    string lastDecision, position;
    getInput("decision_in", lastDecision);
    getInput("position", position);

    double kickDir = brain->data->kickDir; // 현재 골대 중심 방향
    double dir_rb_f = brain->data->robotBallAngleToField; // 로봇 -> 공 벡터 방향
    auto ball = brain->data->ball;
    double ballRange = ball.range;
    double ballYaw = ball.yawToRobot; // 공이 내 정면에 있나
    double ballX = ball.posToRobot.x;
    double ballY = ball.posToRobot.y;
    double distToGoal = 0.0;
    
    distToGoal = norm(ball.posToField.x - (-brain->config->fieldDimensions.length/2), ball.posToField.y);

    // 장애물 회피 로직 -> 드리블이나 오프더볼 등
    // bool avoidPushing;
    // double kickAoSafeDist;
    // brain->get_parameter("obstacle_avoidance.avoid_during_kick", avoidPushing);
    // brain->get_parameter("obstacle_avoidance.kick_ao_safe_dist", kickAoSafeDist);
    // bool avoidKick = avoidPushing // 전방에 장애물 있나
    //     && brain->data->robotPoseToField.x < brain->config->fieldDimensions.length / 2 - brain->config->fieldDimensions.goalAreaLength
    //     && brain->distToObstacle(brain->data->ball.yawToRobot) < kickAoSafeDist;


    // 변수 로드
    double kickYOffset = -0.077; 
    getInput("kick_y_offset", kickYOffset);

    double setPieceGoalDist = 2.0;
    getInput("set_piece_goal_dist", setPieceGoalDist);

    // 절대값으로 양발 -> 그냥 절대값이면 항상 양수로 들어갈테니 ball.posToRobot.y로 공이 왼쪽에 있는지 오른쪽에 있는지 판단하고 부호 변경
    if (kickYOffset > 0) {
        if (brain->data->ball.posToRobot.y > 0) kickYOffset = fabs(kickYOffset);
        else kickYOffset = -fabs(kickYOffset);
    }

    // 정렬 오차 계산
    double deltaDir = toPInPI(kickDir - dir_rb_f);                              // 로봇이 공 뒤에 일직선으로 서있으면 0이라 생각할 수 있음
    double targetAngleOffset = atan2(kickYOffset, ballRange);                    // 오른 발로 차야하니까 공보다 약간 왼쪽에 있어야함 -> 그 왼쪽에 해당하는 각도
    double errorDir = toPInPI(deltaDir + targetAngleOffset);                     // 공을 차기 위한 위치인가 -> 최종 위치 오차
    double headingError = toPInPI(kickDir - brain->data->robotPoseToField.theta); // 로봇이 골대를 정확히 보고있나 -> 최종 각도 오차

    bool iKnowBallPos = brain->tree->getEntry<bool>("ball_location_known");
    bool tmBallPosReliable = brain->tree->getEntry<bool>("tm_ball_pos_reliable");
    string newDecision;
    auto color = 0xFFFFFFFF; 


    /* ----------------------- 1. 공 찾기 ----------------------- */ 
    if (!(iKnowBallPos || tmBallPosReliable)) {
        newDecision = "find";
        color = 0xFFFFFFFF;
    }
    
    /* ----------------------- 2. OffTheBall ----------------------- */
    else if (!brain->data->tmImLead && ballRange >= 1.0) {
        newDecision = "offtheball";
        color = 0x00FFFFFF;
    } 

    /* ----------------------- 3. 공 chase ----------------------- */
    else if (ballRange > chaseRangeThreshold) {
        newDecision = "chase";
        color = 0x0000FFFF;
    } 

    /* ----------------------- 4. 공 드리블 ----------------------- */
    else if (distToGoal > 3.0) {
        newDecision = "dribble";
        color = 0x00FFFF00; 
    }

    /* ----------------------- 5. 공 슛/정렬 ----------------------- */
    // 멀면 정밀하게
    else {
        double kickTolerance = 0.3; // 로봇이 골대를 얼마나 정확히 보고있나
        double yawTolerance = 0.5;  // 공이 내 발 앞에 있는가? 
        
        // 가까우면(세트피스 거리면) 여유롭게
        if (distToGoal < setPieceGoalDist) {
            kickTolerance = 0.3; 
            yawTolerance = 0.5;
        }

        auto now = brain->get_clock()->now();
        auto dt = brain->msecsSince(timeLastTick);
        bool reachedKickDir = fabs(errorDir) < kickTolerance && fabs(headingError) < kickTolerance && dt < 100; // 정렬 완료 상태 bool 값

        timeLastTick = now;


        /* ----------------------- 6. Kick ----------------------- */
        double kickRange = 1.0;
        if (distToGoal < setPieceGoalDist){
            kickRange = 3.0;
        }

        if (
            (reachedKickDir) 
            && brain->data->ballDetected

            && fabs(brain->data->ball.yawToRobot) < yawTolerance 

            // && !avoidKick

            && ball.range < kickRange
        ) {
            // 골대 거리에 따라 Quick vs Normal Kick 결정
            if (distToGoal < setPieceGoalDist) {
                newDecision = "kick_quick"; 
            }
            else {
                newDecision = "kick";      
            }
            
            color = 0x00FF00FF;
            brain->data->isFreekickKickingOff = false; 
        }
        
        /* ----------------------- 7. Adjust ----------------------- */
        else {
            // 골대 거리에 따라 Quick vs Normal Adjust 결정
            if (distToGoal < setPieceGoalDist) newDecision = "adjust_quick";
            else newDecision = "adjust";

            color = 0xFFFF00FF;
        }
    }

    setOutput("decision_out", newDecision);

    static int tickCount = 0;
    tickCount++;

    brain->log->logToScreen(
        "tree/Decide",
        format(
            "Cnt: %d Decision: %s dist: %.2f range: %.2f errPos: %.2f errHead: %.2f kicking: %d lead: %d", 
            tickCount, newDecision.c_str(), distToGoal, ballRange, errorDir, headingError, (lastDecision.find("kick") != string::npos), brain->data->tmImLead
        ),
        color
    );
    return NodeStatus::SUCCESS;
}