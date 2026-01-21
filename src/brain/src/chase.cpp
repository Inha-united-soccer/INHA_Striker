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

    // robot(opponent)도 추가
    // auto robots = brain->data->getRobots();
    // obstacles.insert(obstacles.end(), robots.begin(), robots.end());
    

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
        brain->data->robotPoseToField.x - (brain->config->fieldDimensions.length / 2),
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

    // 거리 단계별로 목표 속도 결정 (먼 거리: 빠르게, 근거리: 천천히)
    double targetSpeed = maxSpeed;
    if (dist > slowDistFar) {
         // 공과 멀리 떨어져 있으면 가속
        targetSpeed = maxSpeed;
        log("Phase: Far (Fast)");
    } else {
        // 공이 가까워지면 컨트롤을 위해 감속
        targetSpeed = minSpeed;
        log("Phase: Near (Slow)");
    }

    // 공을 향한 단위 벡터에 targetSpeed를 곱해 원하는 속도 벡터 생성
    double vx = targetSpeed * cos(ballYaw);
    double vy = targetSpeed * sin(ballYaw);
    
    double vtheta = ballYaw; 

    // 회전 각도가 큰 경우 전진 속도를 절반으로 낮춰 회전 안정성 확보
    if (fabs(ballYaw) > 0.5) {
        vx *= 0.5;
        vy *= 0.5;
    }

    // 위 계산은 참고용이며, 실제로는 로봇 좌표계에서 공까지의 벡터를 사용
    vx = brain->data->ball.posToRobot.x;
    vy = brain->data->ball.posToRobot.y;
    
    // 로봇 기준 공 위치를 단위 벡터로 정규화한 뒤 targetSpeed로 스케일링
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
    double goalX = (fd.length / 2.0);

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
    // person도 opponent로 추가
    auto persons = brain->data->getPersons();
    obstacles.insert(obstacles.end(), persons.begin(), persons.end());
    // robot(opponent)도 추가
    auto robots = brain->data->getRobots();
    obstacles.insert(obstacles.end(), robots.begin(), robots.end());
    
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
        // 중앙 고집을 줄여서(0.5->0.3->0.2) 장애물이 있으면 측면으로 더 쉽게 빠지게 함
        double centerPenalty = distFromCenter * 0.2; 
        
        // 골 유효 범위 가산점
        double goalBonus = 0.0;
        if (distFromCenter < fd.goalWidth / 2.0 - 0.2) {
             goalBonus = 2.0; // 골대 안쪽으로 향하는 경로라면 강하게
        }

        // 점수 계산방식
        double obstacleScore = 0.0;
        if (clearance < 0.7) { // 0.6 -> 0.7 (안전 거리 확보)
            // 70cm 이내에 obstacle있으면 크게 감소
            obstacleScore = -100.0 + clearance * 10.0; 
        } 
        else {
            // 장애물 회피 가중치 대폭 증가 (4.0 -> 10.0)
            obstacleScore = clearance * 10.0; 
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
    // double distToGoalCenter = hypot(goalX - ballPos.x, -ballPos.y);

    // if (ballDistToGoal < distToGoalThresh || distToGoalCenter < distToGoalThresh) {
    //     brain->client->setVelocity(0, 0, 0);
    //     log("Success: Ball reached target distance");
    //     return NodeStatus::SUCCESS;
    // }

    // 정렬 상태 확인 공에서 타겟(BestTarget)을 보는 각도
    double angleBallToGoal = atan2(goalY - ballPos.y, goalX - ballPos.x);

    // 로봇에서 공을 보는 각도
    double angleRobotToBall = atan2(ballPos.y - robotPos.y, ballPos.x - robotPos.x);
    
    // 두 각도 차이 -> 0에 가까울 수록 일직선이라는 뜻이됨
    double alignmentError = fabs(toPInPI(angleBallToGoal - angleRobotToBall));
    // 근거리에서는 작은 위치/각도 노이즈로 alignmentError가 쉽게 튀어서 상태 토글이 발생할 수 있으므로 스무딩
    static double smoothAlignmentError = 0.0;
    smoothAlignmentError = smoothAlignmentError * 0.8 + alignmentError * 0.2;
    
    double vx = 0, vy = 0, vtheta = 0;
    string phase = "Align";

    // 드리블 속도 조절 (DribbleChase와 동일한 규칙: 멀면 가속, 가까우면 감속)
    double ballRange = brain->data->ball.range;
    double targetSpeed = maxSpeed;
    if (ballRange > slowDistFar) {
         // 공과의 거리가 slowDistFar보다 크면 최대 속도로 접근
        targetSpeed = maxSpeed;
        log("Phase: Far (Fast)");
    } else {
        // 임계거리 이내에서는 컨트롤을 위해 최소 속도로 유지
        targetSpeed = minSpeed;
        log("Phase: Near (Slow)");
    }

    // 드리블 로직
    double pushDir = 0.0;
    
    static bool isCircleBack = false;
    double enterThresh = deg2rad(30);
    double exitThresh = deg2rad(20);
    // 공이 매우 가까우면 CircleBack으로 빠지는 것이 오히려 멈춤/후진처럼 보여서, 근거리에서는 Push를 우선
    const double closeBallNoCircleBack = 0.22; // m
    
    if (ballRange < closeBallNoCircleBack) {
        isCircleBack = false;
    } else {
        if (smoothAlignmentError > enterThresh) isCircleBack = true;
        else if (smoothAlignmentError < exitThresh) isCircleBack = false;
    }
    
    if (isCircleBack) {
        // [CircleBack 단계] 공을 중심으로 반원 궤적을 그리며 공 뒤(desiredAngle)로 진입
        phase = "CircleBack";
        
        // 1. 목표 거리 설정: circleBackDist와 현재 거리 중 작은 값 사용, 단 최소 0.3m 확보
        double tightCircleBackDist = min(circleBackDist, ballRange);
        if (tightCircleBackDist < 0.3) tightCircleBackDist = 0.3; 

        // 2. 각도 계산: angleBallToRobot(공→로봇) 대비 desiredAngle(공→골대의 반대편)으로 이동
        double angleBallToRobot = atan2(robotPos.y - ballPos.y, robotPos.x - ballPos.x); 
        double desiredAngle = angleBallToGoal + M_PI; 
        
        // 3. 거리 제어: distError(목표-현재)에 비례한 v_radial로 공과 일정 간격 유지
        double distToBall = hypot(ballPos.x - robotPos.x, ballPos.y - robotPos.y);
        double distError = tightCircleBackDist - distToBall; 
        double v_radial = distError * 2.0;
        // CircleBack에서 과도한 후진(멈춤처럼 보이는 백킹) 방지: 반지름 제어 성분의 후진을 제한
        v_radial = cap(v_radial, 0.25, -0.10);
        
        // 4. 접선 제어: angleError(원하는 각-현재 각)에 비례한 v_tangential로 공 주위를 회전
        double angleError = toPInPI(desiredAngle - angleBallToRobot);
        double v_tangential = angleError * 1.5; 
        
        // 5. 필드 속도 계산: v_radial은 공→로봇 방향, v_tangential은 수직(-sin, cos) 성분
        double vX_radial = v_radial * cos(angleBallToRobot);
        double vY_radial = v_radial * sin(angleBallToRobot);
        
        double vX_tangential = v_tangential * -sin(angleBallToRobot);
        double vY_tangential = v_tangential * cos(angleBallToRobot);
        
        // 두 성분을 합산해 최종 필드 속도 산출
        double vX_field = vX_radial + vX_tangential;
        double vY_field = vY_radial + vY_tangential;

        // 6. 속도 제한: 합성 속도가 maxSpeed 초과 시 비율 축소
        double speed = hypot(vX_field, vY_field);
        if (speed > maxSpeed) {
             vX_field *= (maxSpeed / speed);
             vY_field *= (maxSpeed / speed);
        }

        // 7. 로봇 좌표계 변환 및 회전 제어: 필드 속도를 로봇 기준(vx, vy)으로 변환, 헤딩은 공을 향하도록 설정
        vx = cos(robotTheta) * vX_field + sin(robotTheta) * vY_field;
        vy = -sin(robotTheta) * vX_field + cos(robotTheta) * vY_field;
        
        // 헤딩은 공을 바라보도록 회전 (게인 3.0)
        vtheta = brain->data->ball.yawToRobot * 3.0; 
    } 
    else {
        // [Push 단계] 정렬이 완료되었으므로, 공을 골대 방향으로 밀고 나감
        phase = "Push";
        
        // 1. 미는 방향: 로봇 기준 공 방향(pushDir)으로 직진
        pushDir = atan2(brain->data->ball.posToRobot.y, brain->data->ball.posToRobot.x);
        
        // 2. 전진 속도: pushDir 단위벡터에 targetSpeed를 곱해 전진
        vx = targetSpeed * cos(pushDir);
        vy = targetSpeed * sin(pushDir);
        
        // 3. 회전 속도: 공이 정면에 오도록 회전(근거리에서 pushDir 기반 회전이 튀면 토글이 유발될 수 있어 ballYaw 기반 사용)
        vtheta = brain->data->ball.yawToRobot * 2.0;
        
        // if (alignmentError > deg2rad(20)) { ... }
    }

    // 속도 제한
    vx = cap(vx, vxLimit, -vxLimit);
    vy = cap(vy, vyLimit, -vyLimit);
    vtheta = cap(vtheta, vthetaLimit, -vthetaLimit);

    brain->client->setVelocity(vx, vy, vtheta);

    // 디버깅
    if (vx < 0 && phase == "CircleBack") phase += "(Backing)";
    log(format("Phase:%s DistToGoal:%.2f AlignErr:%.1f(%.1f) BallRange:%.2f",
        phase.c_str(), ballDistToGoal, rad2deg(alignmentError), rad2deg(smoothAlignmentError), ballRange));
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


