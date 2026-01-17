#include "brain.h"
#include "chase.h"
#include "brain_tree.h"

#include <cstdlib>
#include <ctime>

// BehaviorTree Factory에 Test 노드를 생성하는 함수를 등록하는 역할 -> 코드 양 줄일 수 있음
#define REGISTER_CHASE_BUILDER(Name)     \
    factory.registerBuilder<Name>( \
        #Name,                     \
        [brain](const string &name, const NodeConfig &config) { return make_unique<Name>(name, config, brain); });


void RegisterChaseNodes(BT::BehaviorTreeFactory &factory, Brain* brain){
    REGISTER_CHASE_BUILDER(SimpleChase) // obstacle 없이 chase만 
    REGISTER_CHASE_BUILDER(Chase) // obstacle 추가된 chase
    REGISTER_CHASE_BUILDER(DribbleChase) // 드리블 전용 chase
    REGISTER_CHASE_BUILDER(DribbleToGoal) // 골대 드리블
    // REGISTER_CHASE_BUILDER(DribbleFigureEight) // 8자 드리블

}

NodeStatus SimpleChase::tick(){
    double stopDist, stopAngle, vyLimit, vxLimit;
    getInput("stop_dist", stopDist); // 공과의 거리 임계값 -> 멈추기 위해
    getInput("stop_angle", stopAngle); // 공과의 각도 임계값 -> 멈추기 위해
    getInput("vx_limit", vxLimit); // x축 속도 제한
    getInput("vy_limit", vyLimit); // y축 속도 제한

    // 공의 위치를 모를 때 
    if (!brain->tree->getEntry<bool>("ball_location_known")){
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    // 로봇 기준 공과의 거리 
    // 단순 P 제어
    double vx = brain->data->ball.posToRobot.x; // 공과의 x축 거리
    double vy = brain->data->ball.posToRobot.y; // 공과의 y축 거리
    double vtheta = brain->data->ball.yawToRobot * 4.0; // 공과의 각도

    // 가까워질수록 속도가 줄어들도록
    double linearFactor = 1 / (1 + exp(3 * (brain->data->ball.range * fabs(brain->data->ball.yawToRobot)) - 3)); 
    vx *= linearFactor;
    vy *= linearFactor;

    // 속도 제한
    vx = cap(vx, vxLimit, -1.0);    
    vy = cap(vy, vyLimit, -vyLimit); 

    if (brain->data->ball.range < stopDist){
        vx = 0;
        vy = 0;
    }

    brain->client->setVelocity(vx, vy, vtheta, false, false, false);
    return NodeStatus::SUCCESS;
}


NodeStatus Chase::tick(){
    auto log = [=](string msg) {
        brain->log->setTimeNow();
        brain->log->log("debug/Chase4", rerun::TextLog(msg));
    };
    log("ticked");

    if (brain->tree->getEntry<string>("striker_state") != "chase") return NodeStatus::SUCCESS;
    
    double vxLimit, vyLimit, vthetaLimit, dist, safeDist;
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    getInput("vtheta_limit", vthetaLimit);
    getInput("dist", dist);
    getInput("safe_dist", safeDist);

    bool avoidObstacle = true;
    brain->get_parameter("obstacle_avoidance.avoid_during_chase", avoidObstacle);
    double oaSafeDist = 1.5;
    brain->get_parameter("obstacle_avoidance.chase_ao_safe_dist", oaSafeDist);

    if (
        brain->config->limitNearBallSpeed
        && brain->data->ball.range < brain->config->nearBallRange
    ) {
        vxLimit = min(brain->config->nearBallSpeedLimit, vxLimit);
    }

    double ballRange = brain->data->ball.range;
    double ballYaw = brain->data->ball.yawToRobot;
    double kickDir = brain->data->kickDir;

    double theta_br = atan2(
        brain->data->robotPoseToField.y - brain->data->ball.posToField.y,
        brain->data->robotPoseToField.x - brain->data->ball.posToField.x
    );
    double theta_rb = brain->data->robotBallAngleToField;
    auto ballPos = brain->data->ball.posToField;


    double vx, vy, vtheta;
    Pose2D target_f, target_r; 
    static string targetType = "direct"; 
    static double circleBackDir = 1.0; 
    double dirThreshold = M_PI / 2;

    // one_touch 상황이라면 몸을 골대쪽으로 할 수 있게
    double distToGoal = norm(
        brain->data->robotPoseToField.x - (-brain->config->fieldDimensions.length / 2),
        brain->data->robotPoseToField.y
    );
    if (distToGoal < 2.0) {
        dirThreshold = M_PI / 6; // 30 degrees
    }

    if (targetType == "direct") dirThreshold *= 1.2;


    // calculate target point
    if (fabs(toPInPI(kickDir - theta_rb)) < dirThreshold) {
        log("targetType = direct");
        targetType = "direct";
        
        target_f.x = ballPos.x - dist * cos(kickDir);
        target_f.y = ballPos.y - dist * sin(kickDir);
    } 
    else {
        // targetType = "circle_back";
        double cbDirThreshold = 0.0; 
        cbDirThreshold -= 0.2 * circleBackDir; 
        circleBackDir = toPInPI(theta_br - kickDir) > cbDirThreshold ? 1.0 : -1.0;
        log(format("targetType = circle_back, circleBackDir = %.1f", circleBackDir));
        double tanTheta = theta_br + circleBackDir * acos(min(1.0, safeDist/max(ballRange, 1e-5))); 
        target_f.x = ballPos.x + safeDist * cos(tanTheta);
        target_f.y = ballPos.y + safeDist * sin(tanTheta);
    }
    target_r = brain->data->field2robot(target_f);
    brain->log->setTimeNow();
    brain->log->logBall("field/chase_target", Point({target_f.x, target_f.y, 0}), 0xFFFFFFFF, false, false);
            
    double targetDir = atan2(target_r.y, target_r.x);
    double distToObstacle = brain->distToObstacle(targetDir);
    
    if (avoidObstacle && distToObstacle < oaSafeDist) {
        log("avoid obstacle");
        auto avoidDir = brain->calcAvoidDir(targetDir, oaSafeDist);
        const double speed = 0.5;
        vx = speed * cos(avoidDir);
        vy = speed * sin(avoidDir);
        vtheta = ballYaw;
        log(format("avoidDir = %.2f", avoidDir));
    } 

    else {
        
        double p_gain = 1.0;
        vx = target_r.x * p_gain;
        vy = target_r.y * p_gain;

        vtheta = ballYaw;   
        
        double speed = sqrt(vx*vx + vy*vy);
        if (speed > vxLimit) {
            vx = vx / speed * vxLimit;
            vy = vy / speed * vxLimit; 
        }
    }

    vx = cap(vx, vxLimit, -vxLimit);
    vy = cap(vy, vyLimit, -vyLimit);
    vtheta = cap(vtheta, vthetaLimit, -vthetaLimit);

    static double smoothVx = 0.0;
    static double smoothVy = 0.0;
    static double smoothVtheta = 0.0;
    smoothVx = smoothVx * 0.7 + vx * 0.3;
    smoothVy = smoothVy * 0.7 + vy * 0.3;
    smoothVtheta = smoothVtheta * 0.7 + vtheta * 0.3;

    // chase 멈춤 조건
    bool chaseDone = brain->data->ball.range < dist * 1.2 && fabs(toPInPI(kickDir - theta_rb)) < M_PI / 3;
    if (chaseDone){
        brain->tree->setEntry("striker_state", "adjust");
        log("chase -> adjust");
    }
    log(format("distToObstacle = %.2f, targetDir = %.2f", distToObstacle, targetDir));
    
    // brain->client->setVelocity(smoothVx, smoothVy, smoothVtheta, false, false, false);
    brain->client->setVelocity(vx, vy, vtheta, false, false, false);
    return NodeStatus::SUCCESS;
}


// DribbleChase
NodeStatus DribbleChase::tick() {
    auto log = [=](string msg) {
        brain->log->setTimeNow();
        brain->log->log("debug/DribbleChase", rerun::TextLog(msg));
    };
    log("ticked");

    double minSpeed, maxSpeed, slowDistFar, slowDistNear;
    double vxLimit, vyLimit, vthetaLimit;
    
    getInput("min_speed", minSpeed);
    getInput("max_speed", maxSpeed);
    getInput("slow_dist_far", slowDistFar);
    getInput("slow_dist_near", slowDistNear);
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    getInput("vtheta_limit", vthetaLimit);

    if (!brain->tree->getEntry<bool>("ball_location_known")){
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    double dist = brain->data->ball.range;
    double ballYaw = brain->data->ball.yawToRobot;

    // Fast -> Slow -> Fast -> Slow
    double targetSpeed = maxSpeed;
    if (dist > slowDistFar) {
         // 멀리 있을 때는 빠르게 접근
        targetSpeed = maxSpeed;
        log("Phase: Far (Fast)");
    } else {
        // 가까워지면 다시 천천히
        targetSpeed = minSpeed;
        log("Phase: Near (Slow)");
    }

    // 목표 속도 벡터 계산 (공 방향으로)
    double vx = targetSpeed * cos(ballYaw);
    double vy = targetSpeed * sin(ballYaw);
    
    double vtheta = ballYaw; 

    // 회전이 많이 필요하면 전진 속도 줄임
    if (fabs(ballYaw) > 0.5) {
        vx *= 0.5;
        vy *= 0.5;
    }

    // 다만 로봇 좌표계 기준이므로:
    vx = brain->data->ball.posToRobot.x;
    vy = brain->data->ball.posToRobot.y;
    
    // 정규화 후 targetSpeed 적용
    double currDist = norm(vx, vy);
    if (currDist > 1e-5) {
        vx = (vx / currDist) * targetSpeed;
        vy = (vy / currDist) * targetSpeed;
    }

    vx = cap(vx, vxLimit, -vxLimit);
    vy = cap(vy, vyLimit, -vyLimit);
    vtheta = cap(vtheta, vthetaLimit, -vthetaLimit);

    brain->client->setVelocity(vx, vy, vtheta);

    log(format("dist: %.2f, speed: %.2f", dist, targetSpeed));

    return NodeStatus::SUCCESS;
}


NodeStatus DribbleToGoal::tick() {
    auto log = [=](string msg) {
        brain->log->setTimeNow();
        brain->log->log("debug/DribbleToGoal", rerun::TextLog(msg));
    };
    log("ticked");

    double vxLimit, vyLimit, vthetaLimit, distToGoalThresh;
    double minSpeed, maxSpeed, slowDistFar, slowDistNear;
    
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    getInput("vtheta_limit", vthetaLimit);
    getInput("dist_to_goal", distToGoalThresh);
    getInput("min_speed", minSpeed);
    getInput("max_speed", maxSpeed);
    getInput("slow_dist_far", slowDistFar);
    getInput("slow_dist_near", slowDistNear);
    double circleBackDist = 0.5;
    getInput("circle_back_dist", circleBackDist);

    if (!brain->tree->getEntry<bool>("ball_location_known")){
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    auto fd = brain->config->fieldDimensions;
    Point ballPos = brain->data->ball.posToField;
    Point robotPos = {brain->data->robotPoseToField.x, brain->data->robotPoseToField.y, 0.0};
    double robotTheta = brain->data->robotPoseToField.theta;

    // 골대 중앙 좌표
    double goalX = -(fd.length / 2.0);

    vector<double> candidatesY;
    double step = 0.25; 
    double halfField = fd.width / 2.0 - 0.5; // offtheball과 동일
    
    for (double y = -halfField; y <= halfField; y += step) { // 마찬가지로 동일
        candidatesY.push_back(y);
    }
    
    double kickDir = brain->data->kickDir;
    if (isfinite(tan(kickDir))) {
        double kickDirTargetY = ballPos.y + tan(kickDir) * (goalX - ballPos.x);
        kickDirTargetY = cap(kickDirTargetY, halfField, -halfField);
        candidatesY.push_back(kickDirTargetY); // 킥방향도 넣기 (킥)
    }

    double targetGoalY = 0.0;
    double bestScore = -1000.0;
    
    auto obstacles = brain->data->getObstacles();
    
    // 가장 좋은 점수의 경로 찾기
    for(double candY : candidatesY) {

        // 장애물 회피 점수
        double clearance = 100.0; // 점수 초기화
        
        // 목표점 지정
        double dx = goalX - ballPos.x; 
        double dy = candY - ballPos.y; 
        double pathLen = hypot(dx, dy); // 경로의 전체 길이
        
        for(const auto& obs : obstacles) {
             if (obs.posToField.x > ballPos.x + 0.8) continue; // 공보다 뒤에 있는 장애물 무시
             
             // 장애물을 경로(선분)에 투영하여 최단 거리 계산
             double t = ((obs.posToField.x - ballPos.x) * dx + (obs.posToField.y - ballPos.y) * dy) / (dx*dx + dy*dy);
             t = max(0.0, min(1.0, t)); // 선분 내부로 제한 (0.0 ~ 1.0)
             
             double closestX = ballPos.x + t * dx; // 경로상에서 장애물과 가장 가까운 점 X
             double closestY = ballPos.y + t * dy; // 경로상에서 장애물과 가장 가까운 점 Y
             double d = hypot(obs.posToField.x - closestX, obs.posToField.y - closestY); // 그 점과 장애물 사이의 거리

             // 가장 가까운 장애물 거리를 clearance로 저장
             if (d < clearance) clearance = d;
        }

        // 골대 중앙 선호 점수
        double distFromCenter = fabs(candY); // 골대 중앙(Y=0)에서 얼마나 떨어져 있는지 확인
        double centerPenalty = distFromCenter * 0.5; // 멀어질수록 낮아짐
        
        // 골 유효 범위 가산점
        double goalBonus = 0.0;
        if (distFromCenter < fd.goalWidth / 2.0 - 0.2) {
             goalBonus = 2.0; // 골대 안쪽으로 향하는 경로라면 강하게
        }

        // 점수 계산방식
        double obstacleScore = 0.0;
        if (clearance < 0.6) {
            // 60cm 이내에 obstacle있으면 크게 감소
            obstacleScore = -100.0 + clearance * 10.0; 
        } 
        else {
            // 안전하다면 장애물과 멀수록 좋음
            obstacleScore = clearance * 2.0; 
        }

        // 최종 점수 = 장애물 점수 - 중앙 이탈 감점 + 골 보너스
        double finalScore = obstacleScore - centerPenalty + goalBonus;

        if (finalScore > bestScore) {
            bestScore = finalScore;
            targetGoalY = candY;
        }
    }

    double goalY = targetGoalY;
    
    // Rerun Visualization of Candidate & Winner
    brain->log->log("debug/dribble_target", 
         rerun::Points2D({{(float)goalX, (float)goalY}})
         .with_colors({0x00FF00FF})
         .with_labels({"BestTarget"})
         .with_radii({0.2f})
    );

    brain->log->log("debug/dribble_arrow", 
        rerun::Arrows2D::from_vectors({{(float)(goalX - ballPos.x), (float)(goalY - ballPos.y)}})
            .with_origins({{(float)ballPos.x, (float)ballPos.y}})
            .with_colors({0x00FFFF00}) // Cyan/Yellowish
            .with_labels({"Dribble Path"})
    );

    // 공과 골대 사이의 거리
    double ballDistToGoal = hypot(goalX - ballPos.x, goalY - ballPos.y);

    // 도달 확인
    double distToGoalCenter = hypot(goalX - ballPos.x, -ballPos.y);

    if (ballDistToGoal < distToGoalThresh || distToGoalCenter < distToGoalThresh) {
        brain->client->setVelocity(0, 0, 0);
        log("Success: Ball reached target distance");
        return NodeStatus::SUCCESS;
    }

    // 정렬 상태 확인 공에서 타겟(BestTarget)을 보는 각도
    double angleBallToGoal = atan2(goalY - ballPos.y, goalX - ballPos.x);

    // 로봇에서 공을 보는 각도
    double angleRobotToBall = atan2(ballPos.y - robotPos.y, ballPos.x - robotPos.x);
    
    // 두 각도 차이 -> 0에 가까울 수록 일직선이라는 뜻이됨
    double alignmentError = fabs(toPInPI(angleBallToGoal - angleRobotToBall));
    
    double vx = 0, vy = 0, vtheta = 0;
    string phase = "Align";

    // 드리블 속도 조절 -> dribblechase와 똑같이
    double ballRange = brain->data->ball.range;
    double targetSpeed = maxSpeed;
    if (ballRange > slowDistFar) {
         // 멀리 있을 때는 빠르게 접근
        targetSpeed = maxSpeed;
        log("Phase: Far (Fast)");
    } else {
        // 가까워지면 다시 천천히
        targetSpeed = minSpeed;
        log("Phase: Near (Slow)");
    }

    // 드리블 로직
    double pushDir = 0.0;
    
    if (alignmentError > deg2rad(30)) {
        // 공 뒤에 제대로 서지 못했다면 CircleBack으로 공뒤로 이동할 수 있게
        phase = "CircleBack";

        double currentDist = max(0.25, min(ballRange, 0.45)); 
    
        double tightCircleBackDist = min(circleBackDist, ballRange);
        if (tightCircleBackDist < 0.3) tightCircleBackDist = 0.3; 

        double targetX = ballPos.x - tightCircleBackDist * cos(angleBallToGoal);
        double targetY = ballPos.y - tightCircleBackDist * sin(angleBallToGoal);
        
        double errX = targetX - robotPos.x;
        double errY = targetY - robotPos.y;
        
        // P-Control for CircleBack
        double vX_field = errX * 2.5;
        double vY_field = errY * 2.5;

        double angleBallToRobot = atan2(robotPos.y - ballPos.y, robotPos.x - ballPos.x); // Ball -> Robot 벡터
        double desiredAngle = angleBallToGoal + M_PI; 
        double angleDiff = toPInPI(desiredAngle - angleBallToRobot);

        if (fabs(angleDiff) > deg2rad(10)) {
            double swirlDir = (angleDiff > 0) ? 1.0 : -1.0;
            double swirlStrength = 1.5;
            
            double tanAngle = angleBallToRobot + swirlDir * M_PI / 2.0;
            
            vX_field += swirlStrength * cos(tanAngle);
            vY_field += swirlStrength * sin(tanAngle);
        }

        // circleback 시에 공에 닿는걸 방지
        double distToBall = hypot(ballPos.x - robotPos.x, ballPos.y - robotPos.y);
        double safeDist = 0.55; 
        if (distToBall < safeDist) {
            double repulsionStrength = 4.0 * (safeDist - distToBall); // 가까울수록 강하게 밈
            vX_field += repulsionStrength * cos(angleBallToRobot);
            vY_field += repulsionStrength * sin(angleBallToRobot);
        }
        
        // 속도 제한 (CircleBack은 빠르게)
        double speed = hypot(vX_field, vY_field);
        if (speed > maxSpeed) {
             vX_field *= (maxSpeed / speed);
             vY_field *= (maxSpeed / speed);
        }

        vx = cos(robotTheta) * vX_field + sin(robotTheta) * vY_field;
        vy = -sin(robotTheta) * vX_field + cos(robotTheta) * vY_field;
        vtheta = brain->data->ball.yawToRobot * 3.5; // Turn faster
        
        if (ballRange < 0.2) vx = -0.3; // 너무 가까우면 후진
    } 
    else {
        // 정렬이 잘 되었다면 Push로 공 방향으로 전진
        phase = "Push";
        pushDir = atan2(brain->data->ball.posToRobot.y, brain->data->ball.posToRobot.x);
        
        vx = targetSpeed * cos(pushDir);
        vy = targetSpeed * sin(pushDir);
        vtheta = pushDir * 3.5; // 공 중심 맞추기 (High Gain)
        
        // 정렬 오차가 조금 있어도 그냥 밀고 가도록 패널티 제거
        // if (alignmentError > deg2rad(20)) {
        //     vx *= 0.5;
        //     vy *= 0.5;
        // }
    }

    // 속도 제한
    vx = cap(vx, vxLimit, -vxLimit);
    vy = cap(vy, vyLimit, -vyLimit);
    vtheta = cap(vtheta, vthetaLimit, -vthetaLimit);

    brain->client->setVelocity(vx, vy, vtheta);

    // 디버깅
    log(format("Phase:%s DistToGoal:%.2f AlignErr:%.1f", phase.c_str(), ballDistToGoal, rad2deg(alignmentError)));
    brain->log->log("debug/dribble_goal", rerun::Points2D({{(float)goalX, (float)goalY}}).with_colors({0x00FF00FF}).with_labels({"GoalTarget"}));
    
    // 드리블 방향 시각화
    brain->log->log("debug/dribble_arrow", 
        rerun::Arrows2D::from_vectors({{cos(pushDir), -sin(pushDir)}})
        .with_origins({{brain->data->ball.posToField.x, -brain->data->ball.posToField.y}})
        .with_colors({0xFFFF00FF})
        .with_labels({"DribbleDir"})
    );
    
    return NodeStatus::SUCCESS;
}



// // 승재욱 - 직접 만든 Chase
// NodeStatus Chase::tick(){
//     auto log = [=](string msg) {
//         brain->log->setTimeNow();
//         brain->log->log("debug/Chase4", rerun::TextLog(msg));
//     };
//     log("ticked");
    
//     // [입력 변수 가져오기]
//     double vxLimit, vyLimit, vthetaLimit, dist, safeDist;
//     getInput("vx_limit", vxLimit); 
//     getInput("vy_limit", vyLimit);
//     getInput("vtheta_limit", vthetaLimit);
//     getInput("dist", dist);          // 목표: 공 뒤쪽 거리 (예: 30cm)
//     getInput("safe_dist", safeDist); // 회전 접근 시 안전 거리

//     // 승재욱 추가 -> 킥 준비 false로 초기화
//     brain->tree->setEntry("ready_to_kick", false);
    
//     // [장애물 회피 파라미터]
//     bool avoidObstacle;
//     brain->get_parameter("obstacle_avoidance.avoid_during_chase", avoidObstacle); 
//     double oaSafeDist;
//     brain->get_parameter("obstacle_avoidance.chase_ao_safe_dist", oaSafeDist); 

//     // [속도 제한 (공 근처 감속)]
//     if ( brain->config->limitNearBallSpeed && brain->data->ball.range < brain->config->nearBallRange){
//         vxLimit = min(brain->config->nearBallSpeedLimit, vxLimit);
//     }

//     // [데이터 가져오기]
//     double ballRange = brain->data->ball.range;
//     double ballYaw = brain->data->ball.yawToRobot;
//     auto ballPos = brain->data->ball.posToField;
    
//     // *중요: kickDir은 CalcKickDir에서 절대좌표로 계산된 값을 쓰는 것이 가장 좋습니다.
//     double kickDir = brain->data->kickDir; 
    
//     double theta_rb = brain->data->robotBallAngleToField; // 로봇 -> 공 각도
//     double theta_br = atan2( // 공 -> 로봇 각도
//         brain->data->robotPoseToField.y - ballPos.y,
//         brain->data->robotPoseToField.x - ballPos.x
//     );

//     // =========================================================================
//     // [1. 상태 관리 & 히스테리시스 (State Machine & Hysteresis)]
//     // =========================================================================
    
//     // 상태를 기억하기 위한 static 변수들
//     static string targetType = "circle_back"; // 초기값
//     static double lockedCircleDir = 1.0;      // 회전 방향 기억 (1.0 or -1.0)
//     static double lockedKickDir = 0.0;        // Direct 모드용 킥 방향 기억
//     static bool isTargetLocked = false;       // 타겟 락 여부

//     // 기준 임계값
//     double baseThreshold = M_PI / 2; // 90도
    
//     // 히스테리시스 임계값 설정 (들어갈 땐 빡빡하게, 나올 땐 널널하게)
//     double enterDirectThresh = baseThreshold;       // 예: 90도 이내면 Direct 진입
//     double exitDirectThresh  = baseThreshold + 0.3; // 예: 108도 벗어나야 CircleBack 복귀 (여유폭 0.3 rad)

//     double angleDiff = fabs(toPInPI(kickDir - theta_rb));

//     // [상태 전환 로직]
//     if (targetType == "direct") {
//         log("targetType = direct");
//         // Direct 상태 유지 중: 오차가 exitThreshold를 넘어야만 상태 변경
//         if (angleDiff > exitDirectThresh) {
//             targetType = "circle_back";
//             isTargetLocked = false; // 락 해제
//             log("State Changed: Direct -> CircleBack");
            
//             // CircleBack 진입 시 회전 방향 결정 (여기서 딱 한 번만 정함!)
//             double cbDirThreshold = -0.2 * lockedCircleDir; 
//             lockedCircleDir = toPInPI(theta_br - kickDir) > cbDirThreshold ? 1.0 : -1.0;
//         }
//     } 
//     else { // targetType == "circle_back"
//         // CircleBack 상태 유지 중: 오차가 enterThreshold보다 작아지면 상태 변경
//         if (angleDiff < enterDirectThresh) {
//             targetType = "direct";
//             log("State Changed: CircleBack -> Direct");
            
//             // Direct 진입 시 현재 킥 방향을 기억(Lock)함
//             lockedKickDir = kickDir; 
//             isTargetLocked = true;
//         }
//     }

//     // =========================================================================
//     // [2. 목표 지점(Target) 계산]
//     // =========================================================================
//     Pose2D target_f, target_r;

//     if (targetType == "direct") {
//         // [Direct 모드]
//         // 킥 방향이 미세하게 흔들려도 무시하고, 기억해둔 'lockedKickDir'를 사용
//         // 단, 공이 굴러갈 수 있으니 'ballPos'는 계속 최신값 반영
//         if (!isTargetLocked) { 
//             lockedKickDir = kickDir; 
//             isTargetLocked = true; 
//         }

//         target_f.x = ballPos.x - dist * cos(lockedKickDir);
//         target_f.y = ballPos.y - dist * sin(lockedKickDir);
//     } 
//     else {
//         // [Circle Back 모드]
//         // 회전 방향(lockedCircleDir)을 사용하므로 좌우로 튀지 않음
//         double tanTheta = theta_br + lockedCircleDir * acos(min(1.0, safeDist/max(ballRange, 1e-5))); 
//         target_f.x = ballPos.x + safeDist * cos(tanTheta);
//         target_f.y = ballPos.y + safeDist * sin(tanTheta);
//     }

//     // 로봇 기준 좌표로 변환
//     target_r = brain->data->field2robot(target_f);
    
//     // 디버그 로그 (Rerun)
//     brain->log->setTimeNow();
//     brain->log->logBall("field/chase_target", Point({target_f.x, target_f.y, 0}), 0xFFFFFFFF, false, false);

//     // =========================================================================
//     // [3. 이동 명령 생성 (Movement Generation)]
//     // =========================================================================
    
//     double vx, vy, vtheta;
//     double targetDir = atan2(target_r.y, target_r.x);
//     double distToTarget = norm(target_r.x, target_r.y); // 목표까지 남은 거리
//     double distToObstacle = brain->distToObstacle(targetDir);

//     // (A) 장애물 회피
//     if (avoidObstacle && distToObstacle < oaSafeDist) {
//         log("Avoiding Obstacle");
//         auto avoidDir = brain->calcAvoidDir(targetDir, oaSafeDist);
//         const double speed = 0.5;
//         vx = speed * cos(avoidDir);
//         vy = speed * sin(avoidDir);
//         vtheta = ballYaw;
//     } 
//     // (B) 일반 주행
//     else {
//         double stopTolerance = 0.1; // 10cm 이내 도착 판정
        
//         // [도착 처리] 목표 지점에 거의 다 왔으면
//         if (distToTarget < stopTolerance) {
//             vx = 0.0;
//             vy = 0.0;
            
//             // 도착했으면 이동 방향(targetDir)이 아니라, 공 방향(ballYaw)을 바라봄
//             // 이미 lockedKickDir로 정렬해서 들어왔으므로 ballYaw를 보면 킥 각이 나옴
//             vtheta = ballYaw; 
//             // 각도 오차가 5도(약 0.09 rad) 이내면 회전 모터를 끔 -> 제자리 떨림 방지 핵심
//             if (fabs(vtheta) < 0.2) {
//                 vtheta = 0.0;
//                 // 승재욱 추가 -> 킥 준비
//                 brain->tree->setEntry("ready_to_kick", true);
//             }
//         }
//         // [이동 처리] 아직 가는 중이면
//         else {
//             // 거리에 비례하여 속도 조절 (도착지점에서 부드럽게 감속)
//             vx = min(vxLimit, distToTarget); 
//             vy = 0;
//             vtheta = targetDir;

//             // 먼 거리 직진 보정
//             if (fabs(targetDir) < 0.1 && ballRange > 2.0) vtheta = 0.0;
            
//             // 회전 시 전진 감속 (커브 돌 때 속도 줄임)
//             vx *= sigmoid((fabs(vtheta)), 1, 3); 
//         }
//     }

//     // =========================================================================
//     // [4. 안정화: 데드존 (Deadzone)]
//     // =========================================================================
    
//     // // 각도 오차가 5도(약 0.09 rad) 이내면 회전 모터를 끔 -> 제자리 떨림 방지 핵심
//     // if (fabs(vtheta) < 0.2) {
//     //     vtheta = 0.0;
//     // }

//     // [5. 리미트 적용]
//     vx = cap(vx, vxLimit, -vxLimit);
//     vy = cap(vy, vyLimit, -vyLimit);
//     vtheta = cap(vtheta, vthetaLimit, -vthetaLimit);

//     // [6. 명령 전송 (스무딩 없이 즉시 반응)]
//     brain->client->setVelocity(vx, vy, vtheta, false, false, false);
    
//     return NodeStatus::SUCCESS;
// }


