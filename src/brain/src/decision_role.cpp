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
    
    double targetAngleOffset = atan2(kickYOffset, ballRange);
    double errorDir = toPInPI(deltaDir + targetAngleOffset);

    auto now = brain->get_clock()->now();
    auto dt = brain->msecsSince(timeLastTick);
    bool reachedKickDir = 
        errorDir * lastDeltaDir <= 0 
        && fabs(errorDir) < 0.1 // M_PI/6 (30도) -> 0.1 (5.7도)로 강화
        && dt < 100;
    reachedKickDir = reachedKickDir || fabs(errorDir) < 0.05; // 0.1 (5.7도) -> 0.05 (2.9도)로 강화
    timeLastTick = now;
    lastDeltaDir = deltaDir;
   
    bool maintainKick = (lastDecision == "kick" && fabs(errorDir) < 0.5); 

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
    //                  angleGoodForKick          
    //                  && !avoidKick             
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
    //         && norm(brain->data->robotPoseToField.x - (-brain->config->fieldDimensions.length/2), brain->data->robotPoseToField.y) < 1.2
    //         // 킥 방향(kickDir)과 로봇 방향(dir_rb_f)이 대략적으로 일치할 때 one_touch 실행
    //         && fabs(toPInPI(brain->data->kickDir - brain->data->robotBallAngleToField)) < 0.5
    //     )
    // ) {
    //     newDecision = "one_touch";
    //     color = 0xFF0000FF; // Red color
    // } else if (!brain->data->tmImLead) {
    //     newDecision = "offtheball";
    //     color = 0x00FFFFFF;
    // } 
    else if (ballRange > chaseRangeThreshold * (lastDecision == "chase" ? 0.9 : 1.0))
    {
        newDecision = "chase";
        color = 0x0000FFFF;
    } 
    // 공 소유하고 있고 골대와 멀면 드리블 : 골대와 거리가 멀어도 슈팅 각이 있으면 드리블 대신 슛(Adjust -> Kick) 시도
    // 반대로 이미 세트피스 범위라면 adjust 생략 가능
    
    // 골대 중앙과 로봇 사이 거리 계산
    double distToGoal = norm(brain->data->robotPoseToField.x - (-brain->config->fieldDimensions.length/2), brain->data->robotPoseToField.y);
    // 골대를 향하는 직선거리에 장애물이 있나
    bool isShotPathClear = false;

    // 슛 경로 확인 (4m 이내일 때만 체크)
    if (distToGoal < 4) {
        isShotPathClear = true;
        auto obstacles = brain->data->getObstacles();
        Point goalPos = {-brain->config->fieldDimensions.length / 2.0, 0.0, 0.0}; // 골대 위치
        Point myPos = {brain->data->robotPoseToField.x, brain->data->robotPoseToField.y, 0.0};
        
        double pathDx = goalPos.x - ball.posToField.x;
        double pathDy = goalPos.y - ball.posToField.y;
        double pathLen = hypot(pathDx, pathDy); // 공에서 골대까지 거리
        
        for(const auto& obs : obstacles) {
             if (obs.posToField.x > ball.posToField.x) continue;
             
             // 공위치에서 각 obstacle까지 벡터
             double obsDx = obs.posToField.x - ball.posToField.x;
             double obsDy = obs.posToField.y - ball.posToField.y;
             
             // 투영비율 -> obstacle을 공과 골대 직선 위에 두고 그 위치가 선분 사이에 어디쯤인지 비율로 계산
             // 0<t<1일 때 장애물이 공과 골대사이에 위치하므로 고려
             double t = (obsDx * pathDx + obsDy * pathDy) / (pathLen * pathLen);
             
             if (t > 0.0 && t < 1.0) {
                // 공 - 골대 직선 위에서 장애물이랑 가장 가까운 점 좌표 구하기
                 double closestX = ball.posToField.x + t * pathDx;
                 double closestY = ball.posToField.y + t * pathDy;

                 // 실제 장애물 위치와 경로상 가까운 closest 사이 거리
                 double dist = hypot(obs.posToField.x - closestX, obs.posToField.y - closestY);
                 
                 // 슛 경로가 막혔다고 판단하는 거리
                 if (dist < 0.6) {
                     isShotPathClear = false;
                     break;
                 }
             }
        }
    }

    // 2.5m 보다 멀거나 1.6m보다 멀면서 슛길이 막혀있으면 -> 드리블
    // 2.5m 보다 멀거나 1.6m보다 멀면서 슛길이 막혀있으면 -> 드리블
    // if (distToGoal > 2.5 || (!isShotPathClear && distToGoal > 1.5))
    // {
    //     newDecision = "dribble";
    //     color = 0x00FFFF00; 
    // } 

    if (
        (
            ( (reachedKickDir || maintainKick) && !brain->data->isFreekickKickingOff) 
            // || reachedKickDir
        )
        && brain->data->ballDetected
        && fabs(brain->data->ball.yawToRobot) < M_PI / 2.
        && !avoidKick
        && ball.range < 0.8 // 1.5 -> 0.65로 변경하여 더 가까이서 킥 시작 (타점 개선)
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

    setOutput("decision_out", newDecision);
    brain->log->logToScreen(
        "tree/Decide",
        format(
            "Decision: %s ballrange: %.2f ballyaw: %.2f kickDir: %.2f rbDir: %.2f angleGoodForKick: %d lead: %d", 
            newDecision.c_str(), ballRange, ballYaw, kickDir, dir_rb_f, angleGoodForKick, brain->data->tmImLead
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