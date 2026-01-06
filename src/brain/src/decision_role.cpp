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
    double distToGoal = 0.0; // Initialize for logging logging scope
    
    const double goalpostMargin = 0.3; 
    bool angleGoodForKick = brain->isAngleGood(goalpostMargin, "kick");

    bool avoidPushing;
    double kickAoSafeDist;
    brain->get_parameter("obstacle_avoidance.avoid_during_kick", avoidPushing);
    brain->get_parameter("obstacle_avoidance.kick_ao_safe_dist", kickAoSafeDist);
    bool avoidKick = avoidPushing 
        && brain->data->robotPoseToField.x < brain->config->fieldDimensions.length / 2 - brain->config->fieldDimensions.goalAreaLength
        && brain->distToObstacle(brain->data->ball.yawToRobot) < kickAoSafeDist;

    log(format("ballRange: %.2f, ballYaw: %.2f, ballX:%.2f, ballY: %.2f kickDir: %.2f, dir_rb_f: %.2f, angleGoodForKick: %d",
        ballRange, ballYaw, ballX, ballY, kickDir, dir_rb_f, angleGoodForKick));

    
    double deltaDir = toPInPI(kickDir - dir_rb_f);
    
    // 정렬 기준은 deltaDir가 0이 아니라 offset 각도와 일치하는 것
    double kickYOffset = 0.13; 
    getInput("kick_y_offset", kickYOffset);

    double oneTouchGoalDist = 1.2;
    getInput("one_touch_goal_dist", oneTouchGoalDist);
    
    double targetAngleOffset = atan2(kickYOffset, ballRange);
    double errorDir = toPInPI(deltaDir + targetAngleOffset);

    auto now = brain->get_clock()->now();
    auto dt = brain->msecsSince(timeLastTick);
    
    double headingError = toPInPI(kickDir - brain->data->robotPoseToField.theta);


    bool reachedKickDir = 
        fabs(errorDir) < 0.08
        && fabs(headingError) < 0.08
        && dt < 100;
    
    // reachedKickDir = reachedKickDir || fabs(errorDir) < 0.02; 

    timeLastTick = now;
    lastDeltaDir = deltaDir;
   
    // 킥 동작 중이라도 틀어지면 멈추고 다시 정렬하도록 강화, 기본값보다는 크게 줘야함 -> 얼마나? (추가된 로직)
    bool maintainKick = (lastDecision == "kick" && fabs(errorDir) < 0.15 && fabs(headingError) < 0.15); 

    string newDecision;
    auto color = 0xFFFFFFFF; 
    bool iKnowBallPos = brain->tree->getEntry<bool>("ball_location_known");
    bool tmBallPosReliable = brain->tree->getEntry<bool>("tm_ball_pos_reliable");
    if (!(iKnowBallPos || tmBallPosReliable))
    {
        newDecision = "find";
        color = 0xFFFFFFFF;
    } 
    // 세트피스 상황이거나, 일반 경기에서도 골대랑 가까우면 one_touch
    // else if (
    //     (
    //         (
    //             (
    //                 (brain->tree->getEntry<string>("gc_game_sub_state_type") == "CORNER_KICK"
    //                 || brain->tree->getEntry<string>("gc_game_sub_state_type") == "GOAL_KICK"
    //                 || brain->tree->getEntry<string>("gc_game_sub_state_type") == "DIRECT_FREE_KICK"
    //                 || brain->tree->getEntry<string>("gc_game_sub_state_type") == "THROW_IN")
    //                 && brain->tree->getEntry<bool>("gc_is_sub_state_kickoff_side")
    //             )
    //             || (
    //                 angleGoodForKick          
    //                 && !avoidKick             
    //             )
    //         )
    //         && brain->data->ballDetected
    //         && ball.range < 0.5 
    //         && fabs(brain->data->ball.yawToRobot) < 0.3 
    //     )
    //     || // 일반 경기 + 골대 근처
    //     (
    //         brain->data->ballDetected
    //         && ball.range < 0.5 
    //         && fabs(brain->data->ball.yawToRobot) < 0.3 
    //         && norm(brain->data->robotPoseToField.x - (-brain->config->fieldDimensions.length/2), brain->data->robotPoseToField.y) < oneTouchGoalDist
    //         // 킥 방향(kickDir)과 로봇 방향(dir_rb_f)이 대략적으로 일치할 때 one_touch 실행
    //         && fabs(toPInPI(brain->data->kickDir - brain->data->robotBallAngleToField)) < 0.5
    //     )
    // ) {
    //     newDecision = "one_touch";
    //     color = 0xFF0000FF; // Red color
    // } 
    else if (!brain->data->tmImLead) {
        newDecision = "offtheball";
        color = 0x00FFFFFF;
    } else if (ballRange > chaseRangeThreshold * (lastDecision == "chase" ? 0.9 : 1.0))
    {
        newDecision = "chase";
        color = 0x0000FFFF;
    } 
    // 공 소유하고 있고 골대와 멀면 드리블 : 골대와 거리가 멀어도 슈팅 각이 있으면 드리블 대신 슛(Adjust -> Kick) 시도
    // 반대로 이미 세트피스 범위라면 adjust 생략 가능
    
    else {
        distToGoal = norm(ball.posToField.x - (-brain->config->fieldDimensions.length/2), ball.posToField.y);

        // 2.0m 보다 멀거나 1.5m보다 멀면서 슛길이 막혀있으면 -> 드리블
        bool shotPathBlocked = false; // 현재 경로 계산은 제거되었으므로 항상 false
        if (distToGoal > 22.0)
        {
            newDecision = "dribble";
            color = 0x00FFFF00; 
        } 

    else if (
        ((reachedKickDir || maintainKick) && !brain->data->isFreekickKickingOff) 
        && brain->data->ballDetected
        && fabs(brain->data->ball.yawToRobot) < M_PI / 2.
        && !avoidKick
        && ball.range < 0.8
    ) {
        // if (brain->data->kickType == "cross") newDecision = "cross";
        // else newDecision = "kick";      
        newDecision = "kick"; // Striker는 Cross 없이 무조건 슛
        color = 0x00FF00FF;
        brain->data->isFreekickKickingOff = false; 
    }
    else
    {
        newDecision = "adjust";
        color = 0xFFFF00FF;
    }
    }

    setOutput("decision_out", newDecision);
    brain->log->logToScreen(
        "tree/Decide",
        format(
            "Decision: %s dist: %.2f errPos: %.2f errHead: %.2f kickDir: %.2f", 
            newDecision.c_str(), distToGoal, errorDir, headingError, kickDir
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