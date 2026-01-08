#include "brain.h"
#include "decision_role.h"
#include "brain_tree.h"

#include <cstdlib>
#include <ctime>

#define REGISTER_DECISION_ROLE_BUILDER(Name)     \
    factory.registerBuilder<Name>( \
        #Name,                     \
        [brain](const string &name, const NodeConfig &config) { return make_unique<Name>(name, config, brain); });


void RegisterDecisionRoleNodes(BT::BehaviorTreeFactory &factory, Brain* brain){
    REGISTER_DECISION_ROLE_BUILDER(StrikerDecide)
    REGISTER_DECISION_ROLE_BUILDER(GoalieDecide)
    REGISTER_DECISION_ROLE_BUILDER(DefenderDecide)
}

NodeStatus StrikerDecide::tick() {
    auto log = [=](string msg) {
        brain->log->setTimeNow();
        brain->log->log("debug/striker_decide", rerun::TextLog(msg));
    };

    double chaseRangeThreshold;
    getInput("chase_threshold", chaseRangeThreshold);
    string lastDecision, position;
    getInput("decision_in", lastDecision);
    getInput("position", position);

    double kickDir = brain->data->kickDir;
    double dir_rb_f = brain->data->robotBallAngleToField; 
    auto ball = brain->data->ball;
    double ballRange = ball.range;
    double ballYaw = ball.yawToRobot;
    double ballX = ball.posToRobot.x;
    double ballY = ball.posToRobot.y;
    double distToGoal = 0.0;
    
    // 거리 계산 (Localization vs Vision)
    distToGoal = norm(ball.posToField.x - (-brain->config->fieldDimensions.length/2), ball.posToField.y);
    auto gps = brain->data->getGoalposts();
    double visibleMinDist = 999.0;
    for(const auto& gp : gps){
        if (gp.posToRobot.x > 0 && gp.range < visibleMinDist) visibleMinDist = gp.range;
    }
    if(visibleMinDist < 10.0 && visibleMinDist < distToGoal) distToGoal = visibleMinDist;
    brain->log->logToScreen("debug/DistCheck", format("DistToGoal: %.2f (Vis: %.2f)", distToGoal, visibleMinDist), 0x00FFFF00);


    // 장애물 회피 로직
    bool avoidPushing;
    double kickAoSafeDist;
    brain->get_parameter("obstacle_avoidance.avoid_during_kick", avoidPushing);
    brain->get_parameter("obstacle_avoidance.kick_ao_safe_dist", kickAoSafeDist);
    bool avoidKick = avoidPushing 
        && brain->data->robotPoseToField.x < brain->config->fieldDimensions.length / 2 - brain->config->fieldDimensions.goalAreaLength
        && brain->distToObstacle(brain->data->ball.yawToRobot) < kickAoSafeDist;


    // 변수 로드
    double kickYOffset = -0.077; 
    getInput("kick_y_offset", kickYOffset);
    double setPieceGoalDist = 2.0;
    getInput("set_piece_goal_dist", setPieceGoalDist);
    
    // 정렬 오차 계산
    double deltaDir = toPInPI(kickDir - dir_rb_f);
    double targetAngleOffset = atan2(kickYOffset, ballRange);
    double errorDir = toPInPI(deltaDir + targetAngleOffset);
    double headingBias = -targetAngleOffset * 0.3; 
    double desiredHeading = kickDir + headingBias;
    double headingError = toPInPI(desiredHeading - brain->data->robotPoseToField.theta);


    bool iKnowBallPos = brain->tree->getEntry<bool>("ball_location_known");
    bool tmBallPosReliable = brain->tree->getEntry<bool>("tm_ball_pos_reliable");
    string newDecision;
    auto color = 0xFFFFFFFF; 


    /* ----------------- 1. 공 찾기 ----------------- */ 
    if (!(iKnowBallPos || tmBallPosReliable)) {
        newDecision = "find";
        color = 0xFFFFFFFF;
    }
    
    /* ----------------- 2. OffTheBall ----------------- */
    else if (!brain->data->tmImLead && ballRange > 0.9) {
        newDecision = "offtheball";
        color = 0x00FFFFFF;
    } 

    /* ----------------- 3. 공 chase ----------------- */
    else if (ballRange > chaseRangeThreshold * (lastDecision == "chase" ? 0.9 : 1.0)) {
        newDecision = "chase";
        color = 0x0000FFFF;
    } 

    /* ----------------- 4. 공 드리블 ----------------- */
    else if (distToGoal > 3.0) {
        newDecision = "dribble";
        color = 0x00FFFF00; 
    }

    /* ----------------- 5. 공 슛/정렬 ----------------- */
    else {
        double kickTolerance = 0.05;
        auto now = brain->get_clock()->now();
        auto dt = brain->msecsSince(timeLastTick);
        bool reachedKickDir = fabs(errorDir) < kickTolerance && fabs(headingError) < kickTolerance && dt < 100;
        bool maintainKick = (lastDecision == "kick" || lastDecision == "kick_quick") && fabs(errorDir) < 0.10 && fabs(headingError) < 0.10;

        timeLastTick = now;
        lastDeltaDir = deltaDir;

        // 정렬 완료 & 장애물 없음 & 공 가까움
        if (
            ((reachedKickDir || maintainKick) && !brain->data->isFreekickKickingOff) 
            && brain->data->ballDetected
            && fabs(brain->data->ball.yawToRobot) < M_PI / 2.
            && !avoidKick
            && ball.range < 0.8
        ) {
            // 골대 거리(setPieceGoalDist)에 따라 Quick vs Normal Kick 결정
            if (distToGoal < setPieceGoalDist) newDecision = "kick_quick"; 
            else newDecision = "kick";      
            
            color = 0x00FF00FF;
            brain->data->isFreekickKickingOff = false; 
        }

        else {
            // 골대 거리에 따라 Quick vs Normal Adjust 결정
            if (distToGoal < setPieceGoalDist) newDecision = "adjust_quick";
            else newDecision = "adjust";

            color = 0xFFFF00FF;
        }
    }

    setOutput("decision_out", newDecision);
    brain->log->logToScreen(
        "tree/Decide",
        format(
            "Decision: %s dist: %.2f errPos: %.2f errHead: %.2f kicking: %d", 
            newDecision.c_str(), distToGoal, errorDir, headingError, (lastDecision.find("kick") != string::npos)
        ),
        color
    );
    return NodeStatus::SUCCESS;
}

NodeStatus GoalieDecide::tick()
{

    double chaseRangeThreshold;
    getInput("chase_threshold", chaseRangeThreshold);
    string lastDecision, position;
    getInput("decision_in", lastDecision);

    double kickDir = atan2(brain->data->ball.posToField.y, brain->data->ball.posToField.x + brain->config->fieldDimensions.length / 2);
    double dir_rb_f = brain->data->robotBallAngleToField;
    auto goalPostAngles = brain->getGoalPostAngles(0.3);
    double theta_l = goalPostAngles[0]; 
    double theta_r = goalPostAngles[1]; 
    bool angleIsGood = (dir_rb_f > -M_PI / 2 && dir_rb_f < M_PI / 2);
    double ballRange = brain->data->ball.range;
    double ballYaw = brain->data->ball.yawToRobot;

    string newDecision;
    auto color = 0xFFFFFFFF; 
    bool iKnowBallPos = brain->tree->getEntry<bool>("ball_location_known");
    bool tmBallPosReliable = brain->tree->getEntry<bool>("tm_ball_pos_reliable");
    if (!(iKnowBallPos || tmBallPosReliable))
    {
        newDecision = "find";
        color = 0x0000FFFF;
    }
    else if (brain->data->ball.posToField.x > 0 - static_cast<double>(lastDecision == "retreat"))
    {
        newDecision = "retreat";
        color = 0xFF00FFFF;
    } else if (ballRange > chaseRangeThreshold * (lastDecision == "chase" ? 0.9 : 1.0))
    {
        newDecision = "chase";
        color = 0x00FF00FF;
    }
    else if (angleIsGood)
    {
        newDecision = "kick";
        color = 0xFF0000FF;
    }
    else
    {
        newDecision = "adjust";
        color = 0x00FFFFFF;
    }

    setOutput("decision_out", newDecision);
    brain->log->logToScreen("tree/Decide",
                            format("Decision: %s ballrange: %.2f ballyaw: %.2f kickDir: %.2f rbDir: %.2f angleIsGood: %d", newDecision.c_str(), ballRange, ballYaw, kickDir, dir_rb_f, angleIsGood),
                            color);
    return NodeStatus::SUCCESS;
}

NodeStatus DefenderDecide::tick() {
    double chaseRangeThreshold;
    getInput("chase_threshold", chaseRangeThreshold);
    string lastDecision;
    getInput("decision_in", lastDecision);

    double kickDir = brain->data->kickDir;
    double dir_rb_f = brain->data->robotBallAngleToField; 
    auto ball = brain->data->ball;
    double ballRange = ball.range;
    double ballYaw = ball.yawToRobot;
    double ballX = ball.posToRobot.x;
    double ballY = ball.posToRobot.y;
    
    // 수비수는 안전하게 걷어내는 것이 목표 (패스)
    // angleGoodForKick은 골대 방향을 보는지 확인하는 함수지만, 
    // 반코트 게임에서는 전방으로 차는 동작(패스/걷어내기)을 위해 그대로 사용합니다.
    const double goalpostMargin = 0.5; 
    bool angleGoodForKick = brain->isAngleGood(goalpostMargin, "kick");

    // 장애물 회피 로직
    bool avoidPushing;
    double kickAoSafeDist;
    brain->get_parameter("obstacle_avoidance.avoid_during_kick", avoidPushing);
    brain->get_parameter("obstacle_avoidance.kick_ao_safe_dist", kickAoSafeDist);
    bool avoidKick = avoidPushing 
        && brain->data->robotPoseToField.x < -2.0 
        && brain->distToObstacle(brain->data->ball.yawToRobot) < kickAoSafeDist;


    double deltaDir = toPInPI(kickDir - dir_rb_f);
    auto now = brain->get_clock()->now();
    auto dt = brain->msecsSince(timeLastTick);
    bool reachedKickDir = 
        deltaDir * lastDeltaDir <= 0 
        && fabs(deltaDir) < M_PI / 6
        && dt < 100;
    reachedKickDir = reachedKickDir || fabs(deltaDir) < 0.1;
    timeLastTick = now;
    lastDeltaDir = deltaDir;

    string newDecision;
    auto color = 0xFFFFFFFF; 
    bool iKnowBallPos = brain->tree->getEntry<bool>("ball_location_known");
    bool tmBallPosReliable = brain->tree->getEntry<bool>("tm_ball_pos_reliable");


    // 1. 공을 모르면 -> 찾기
    if (!(iKnowBallPos || tmBallPosReliable))
    {
        newDecision = "find";
        color = 0xFFFFFFFF;
    } 
    // 2. 추적 거리 밖이면 -> chase
    // else if (ballRange > chaseRangeThreshold * (lastDecision == "chase" ? 0.9 : 1.0))
    // {
    //     // 수비수는 쫓아가지 않고 대기 -> 당장은 pass 이후 등에서 활용됨
    //     if (brain->data->ball.posToField.x < -1.0) {
    //         newDecision = "wait";
    //         color = 0x00FFFFFF; // Cyan/White mix
    //     } else {
    //         newDecision = "chase";
    //         color = 0x0000FFFF;
    //     }
    // } 
    // 3. 킥 조건 만족하면 -> kick(패스)
    else if (
        (
            (angleGoodForKick && !brain->data->isFreekickKickingOff) 
            || reachedKickDir
        )
        && brain->data->ballDetected
        && fabs(brain->data->ball.yawToRobot) < M_PI / 2.
        && !avoidKick
        && ball.range < 1.5
    ) {
        newDecision = "pass"; // 수비수는 킥을 "패스"라고 명명
        color = 0x00FF00FF;
        brain->data->isFreekickKickingOff = false; 
    }
    // 4. 그 외 -> 위치 조정 ("adjust")
    else
    {
        newDecision = "adjust";
        color = 0xFFFF00FF;
    }

    setOutput("decision_out", newDecision);
    brain->log->logToScreen(
        "tree/Defend",
        format(
            "Decision: %s ballrange: %.2f ballyaw: %.2f kickDir: %.2f rbDir: %.2f lead: %d", 
            newDecision.c_str(), ballRange, ballYaw, kickDir, dir_rb_f, brain->data->tmImLead
        ),
        color
    );
    return NodeStatus::SUCCESS;
}