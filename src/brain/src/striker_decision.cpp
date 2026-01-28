#include "brain.h"
#include "brain_tree.h"
#include "striker_decision.h"

#include <cstdlib>
#include <ctime>

// #define REGISTER_STRIKERDECISION_BUILDER(Name)     \
//     factory.registerBuilder<Name>( \
//         #Name,                     \
//         [brain](const string &name, const NodeConfig &config) { return make_unique<Name>(name, config, brain); });


void RegisterStrikerDecisionNodes(BT::BehaviorTreeFactory &factory, Brain* brain){
    REGISTER_STRIKERDECISION_BUILDER(StrikerDecision)
}

NodeStatus StrikerDecision::tick() {
    // --- 1. 전술(Tactic)에서 설정한 파라미터를 읽어오기 ---
    double paramChaseSpeed = 0.8;
    double paramDefenseLineX = -10.0; // 기본값: 제한 없음
    double paramKickThreshold = 0.3;  
    
    // Default 값
    if (auto val = brain->tree->getEntry<double>("Strategy.param_chase_speed_limit")) paramChaseSpeed = val;
    if (auto val = brain->tree->getEntry<double>("Strategy.param_defense_line_x")) paramDefenseLineX = val;
    if (auto val = brain->tree->getEntry<double>("Strategy.param_kick_threshold")) paramKickThreshold = val;

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
    double distToGoal = norm(ball.posToField.x - (brain->config->fieldDimensions.length/2), ball.posToField.y);

    double goalX = brain->config->fieldDimensions.length / 2.0;
    double goalY = brain->config->fieldDimensions.goalWidth / 2.0;

    // 공이 적팀 골대 안으로 들어갔는지 판별
    // x가 골라인보다 크고(즉, 넘어갔고), y가 골대 폭 안에 있으면 골로 인정
    if (ball.posToField.x > goalX && fabs(ball.posToField.y) < goalY) {
        brain->data->hasScored = true;
    }

    // 골을 넣었다면 멈추기
    if (brain->data->hasScored) {
        setOutput("decision_out", string("stop_goal"));
        brain->log->logToScreen("tree/Decide", "GOAL SCORED! Stopping...", 0xFF0000FF);
        return NodeStatus::SUCCESS;
    }

    // 변수 로드
    double setPieceGoalDist = 3.0;
    getInput("set_piece_goal_dist", setPieceGoalDist);


    // 정렬 오차 계산
    double deltaDir = toPInPI(kickDir - dir_rb_f);                  // 로봇이 공 뒤에 일직선으로 서있으면 0이라 생각할 수 있음            
    double errorDir = toPInPI(deltaDir);                            // 공을 차기 위한 위치인가 -> 최종 위치 오차
    double headingError = toPInPI(kickDir - brain->data->robotPoseToField.theta); // 로봇이 골대를 정확히 보고있나 -> 최종 각도 오차

    bool iKnowBallPos = brain->tree->getEntry<bool>("ball_location_known");
    bool tmBallPosReliable = brain->tree->getEntry<bool>("tm_ball_pos_reliable");
    string newDecision;
    auto color = 0xFFFFFFFF; 

    // 팀원 패스 신호 받기
    int myId = brain->config->playerId;
    // Fix logic: if 1 (Striker), partner 2 (Index 1). If 2 (Defender), partner 1 (Index 0).
    int partnerIdx = (myId == 1) ? 1 : 0;
    
    // Check isAlive to prevent stale signals
    bool passsignal = brain->data->tmStatus[partnerIdx].isAlive && brain->data->tmStatus[partnerIdx].passSignal;
    
    // 패스 신호 타임 카운트 (Same)
    bool isReceiveTimeout = false;
    if (passsignal) {
        if (receiveStartTime.nanoseconds() == 0) receiveStartTime = brain->get_clock()->now();
        else if (brain->msecsSince(receiveStartTime) > 5000) isReceiveTimeout = true;
    } else {
        receiveStartTime = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }
    
    // ========================= 의사 결정 로직 =========================
    
    /* ----------------------- 1. 공 찾기 ----------------------- */ 
    if (!(iKnowBallPos || tmBallPosReliable)) {
        newDecision = "find";
        color = 0xFFFFFFFF;
    }


    /* ----------------------- 2. 패스 받기 ----------------------- */ 
    else if (passsignal && !isReceiveTimeout && ballRange > 0.6){
        newDecision = "receive";
        color = 0x00FFFFFF;
    }
    
    /* ----------------------- 3. OffTheBall ----------------------- */
    else if (!brain->data->tmImLead && ballRange >= 1.0) {
        newDecision = "offtheball";
        color = 0x00FFFFFF;
    }

    /* ----------------------- 4. 공 chase ----------------------- */
    else if (ballRange > chaseRangeThreshold) {
        newDecision = "chase";
        color = 0x0000FFFF;
    } 

    /* ----------------------- 5. 공 드리블 ----------------------- */
    else if (distToGoal > 2.0) {
        newDecision = "dribble";
        color = 0x00FFFF00; 
    }

    /* ----------------------- 6. 공 슛/정렬 ----------------------- */
    else {
        // getInput("kick_tolerance", kickTolerance); // xml에서 수정 가능하도록
        // getInput("yaw_tolerance", yawTolerance);
        double kickTolerance = paramKickThreshold; 
        double yawTolerance = 0.5; // Yaw는 넉넉하게
        
        // 가까우면(세트피스 거리면) 여유롭게
        if (distToGoal < setPieceGoalDist) {
            kickTolerance *= 1.5; // 좀 더 여유롭게
            yawTolerance = 0.6;
        }

        auto now = brain->get_clock()->now();
        auto dt = brain->msecsSince(timeLastTick);
        bool reachedKickDir = fabs(errorDir) < kickTolerance && fabs(headingError) < kickTolerance && dt < 100; // 정렬 완료 상태 bool 값

        timeLastTick = now;


        /* ----------------------- 7. Kick Lock Logic ----------------------- */
        static rclcpp::Time kickLockEndTime(0, 0, RCL_ROS_TIME);
        bool isLocked = (now < kickLockEndTime);
        
        // Lock 해제 조건: 공이 너무 멀어지면 즉시 해제
        if (isLocked && ballRange > 0.6) {
            isLocked = false; 
            kickLockEndTime = rclcpp::Time(0, 0, RCL_ROS_TIME);
        }

        double kickRange = 1.0;
        if (distToGoal < setPieceGoalDist) kickRange = 2.0;

        // Kick 진입 조건
        if (
            (reachedKickDir || isLocked) // 정렬됐거나 이미 락이 걸려있으면
            && brain->data->ballDetected
            && (fabs(brain->data->ball.yawToRobot) < yawTolerance || isLocked) // 락 걸려있으면 각도 무시
            // && !avoidKick
            && ball.range < kickRange
        ) {
            if (distToGoal < setPieceGoalDist) newDecision = "kick_quick"; 
            else newDecision = "kick";      
            
            if (!isLocked) kickLockEndTime = now + rclcpp::Duration::from_seconds(3.0);
            
            color = 0x00FF00FF;
            brain->data->isFreekickKickingOff = false; 
        }
        
        /* ----------------------- 8. Adjust ----------------------- */
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
            "Dec: %s (K_Thresh: %.2f) dist: %.2f err: %.2f", 
            newDecision.c_str(), paramKickThreshold, distToGoal, errorDir
        ),
        color
    );
    return NodeStatus::SUCCESS;
}