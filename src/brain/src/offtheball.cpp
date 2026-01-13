#include "brain.h"
#include "offtheball.h"
#include "brain_tree.h"

#include <cstdlib>
#include <ctime>

#define REGISTER_OFFTHEBALL_BUILDER(Name)     \
    factory.registerBuilder<Name>( \
        #Name,                     \
        [brain](const string &name, const NodeConfig &config) { return make_unique<Name>(name, config, brain); });


void RegisterOfftheballNodes(BT::BehaviorTreeFactory &factory, Brain* brain){
    REGISTER_OFFTHEBALL_BUILDER(OfftheballPosition)
}

/*
패스 받기 전 오프더볼 무브 추후 추가할 점
1. 이동 경로상 장애물 회피
2. 공 위치에 따라 이동 경로 변경
3. 통신으로 받은 도착 위치로 이동
4. 골대 충돌 방지
5. 목표위치에서의 속도 조절
*/
NodeStatus OfftheballPosition::tick(){
    auto startTick = std::chrono::high_resolution_clock::now();

    // 경기장 규격 및 파라미터 가져오기
    auto fd = brain->config->fieldDimensions;
    
    // 골대 앞에서 얼마나 떨어져 있을지
    double distFromGoal = 2.0; // 골대에서 떨어진 기준 거리 (2.0m) 
    if (!getInput("dist_from_goal", distFromGoal)) {
        distFromGoal = 2.0;
    }
    
    // 골대 중앙 좌표
    double goalX = -(fd.length / 2.0);
    // 골대에서 distFromGoal만큼 떨어진 좌표
    double baseX = goalX + distFromGoal; // 오프더볼 위치 기준 X좌표 

    // 최적의 Y좌표 계산 (경기장 폭 0.5m씩 안쪽)
    double maxY = fd.width / 2.0 - 0.5;
    double bestX = baseX;
    double bestY = 0.0;

    double maxScore = -1e9;
    
    // 장애물 정보 나중에 사용
    // auto obstacles = brain->data->getObstacles();

    // 최종 위치 계산
    static double lastBestY = 0.0;

    // 현재 로봇 위치
    double robotX = brain->data->robotPoseToField.x;
    double robotY = brain->data->robotPoseToField.y;
    double robotTheta = brain->data->robotPoseToField.theta;

    // 상대선수들
    auto Opponents = brain->data->getRobots();


    std::vector<int> defenderIndices;
    for (size_t idx = 0; idx < Opponents.size(); idx++) {
        if (Opponents[idx].label != "Opponent") continue;

        if (std::abs(Opponents[idx].posToField.x - goalX) < 4.0) {
            defenderIndices.push_back(idx);
        }
    }

    // opponent 대칭점 계산 Y
    double symTargetY = 0.0;
    if (!defenderIndices.empty()) {
        double totalOppY = 0.0;
        for (int idx : defenderIndices) {
            totalOppY += Opponents[idx].posToField.y;
        }
        symTargetY = -(totalOppY / defenderIndices.size());
    }

    // Y, X 축을 따라 0.1m 간격으로 후보 지점 탐색
    for (double x = baseX-2.0; x <= baseX+2.0; x += 0.05) {
        for (double y = -maxY; y <= maxY; y += 0.05) {
            double distToDefender = 0.0;
            double normalizer = (defenderIndices.size() > 0 ? defenderIndices.size() : 1.0);

            for (const auto& defenderIndex : defenderIndices) {
                double dist = norm(y - Opponents[defenderIndex].posToField.y, baseX - Opponents[defenderIndex].posToField.x);
                dist = cap(dist, 3.0, 0.0); // 3m 면 충분히 멀다. 그 이상 떨어져 있다고 cost를 더 주진 않을것
                distToDefender += dist;
            }

            distToDefender /= normalizer;
            
            double score = 0.0;
            score -= (fabs(x - baseX) * 5.0);   // 1. 기준 X좌표 선호 가중치 (5.0)
            score -= (fabs(y) * 5.0);           // 2. Y축 중앙 선호 가중치 (5.0)
            score -= (fabs(x - robotX) * 3.0);  // 3. 현재 위치 유지(X) 가중치 (3.0)
            score -= (fabs(y - robotY) * 3.0);  // 4. 현재 위치 유지(Y) 가중치 (3.0)
            score += (distToDefender * 20.0);   // 5. 수비수와의 거리 확보 가중치 (20.0)

            if (!defenderIndices.empty()) {
                score -= std::abs(y - symTargetY) * 10.0; // 6. 대칭점(빈 공간) 선호 가중치 (10.0)
            }

            double distXToBall = std::abs(x - brain->data->ball.posToField.x);
            score -= std::abs(distXToBall - 2.5) * 3.0; // 7. 공과의 거리(X축 깊이) 2.5m 유지 가중치 (3.0)

            score += (-x) * 5.2; // 8. 공격 방향(전진) 선호 가중치 (5.2)

            Line passPath = {brain->data->ball.posToField.x, brain->data->ball.posToField.y, x, y};
            Line shotPath = {baseX, y, goalX, 0.0};

            for (const auto& opponent : Opponents) {
                if (opponent.label != "Opponent") continue;

                rclcpp::Time now = brain->get_clock()->now();
                double elapsed = (now - opponent.timePoint).seconds();
                double confidenceFactor = std::max(0.0, (5.0 - elapsed) / 5.0);  // 메모리 기반 신뢰도 계산 -> 시간이 지날수록 신뢰도가 떨어지게

                if (confidenceFactor <= 0.0) continue;

                if (brain->data->ballDetected) { 
                    double distToPassPath = pointMinDistToLine({opponent.posToField.x, opponent.posToField.y}, passPath);
                    if (distToPassPath < 1.5) { 
                        score -= (1.5 - distToPassPath) * 15.0 * confidenceFactor; // 9. 공 사이 패스 경로 페널티 (15.0)
                    }
                }

                double distToShotPath = pointMinDistToLine({opponent.posToField.x, opponent.posToField.y}, shotPath); 
                if (distToShotPath < 1.5) { 
                     score -= (1.5 - distToShotPath) * 3.0 * confidenceFactor; // 10. 슈팅 경로 페널티 (3.0)
                }

                Line movementPath = {robotX, robotY, x, y};
                double distRobotTarget = norm(x - robotX, y - robotY);
                if (distRobotTarget > 0.1) {
                     double distToMovementPath = pointMinDistToLine({opponent.posToField.x, opponent.posToField.y}, movementPath);
                     
                     if (distToMovementPath < 1.5) { 
                         score -= (1.5 - distToMovementPath) * 30.0 * confidenceFactor; // 11. 이동 경로 obstacle 페널티 (30.0)
                     }
                }
            }

            double halfGoalW = brain->config->fieldDimensions.goalWidth / 2.0; // 골대 위치: (goalX, +/- goalWidth/2)
            double distToLeftPost = norm(x - goalX, y - halfGoalW);
            double distToRightPost = norm(x - goalX, y + halfGoalW);
            
            if (distToLeftPost < 0.5) {
                score -= (0.5 - distToLeftPost) * 20.0; // 12. 골대 충돌 방지 페널티 (20.0)
            }
            if (distToRightPost < 0.5) {
                score -= (0.5 - distToRightPost) * 20.0; // 12. 골대 충돌 방지 페널티 (20.0)
            }

            if (score > maxScore) {
                maxScore = score;
                bestX = x;
                bestY = y;
            }
        }
    }
    // score loop ends here    
    // TODO: if maxScore is below ?.?, it's not proper to stay at baseX.
    // TODO: in this case, we need whole new logic

    //lastBestY = bestY;

    // 최종 목표 위치 설정
    double targetX = bestX;
    double targetY = bestY;

    // 이동 벡터 계산
    double errX = targetX - robotX; // X축 이동 필요량
    double errY = targetY - robotY; // Y축 이동 필요량

    // 필드 좌표계 계산
    double vX_field = errX * 1.0;
    double vY_field = errY * 1.0;

    // 로봇 좌표계로 변환
    double vx_robot = cos(robotTheta) * vX_field + sin(robotTheta) * vY_field;
    double vy_robot = -sin(robotTheta) * vX_field + cos(robotTheta) * vY_field;
    vx_robot = cap(vx_robot, 1.0, -0.3); // 최대 1m/s, 최소 0.5m/s
    vy_robot = cap(vy_robot, 0.3, -0.3); // 최대 0.5m/s, 최소 0.5m/s

    // 회전 제어
    double targetTheta;
    
    // 오프더볼 상황에서는 공을 주시하는 것이 유리함 하지만 몸은 공을 받을 준비를 하기 위해 공과 골대 사이의 중간 각를 바라보게 함
    double angleToGoal = atan2(0.0 - robotY, goalX - robotX);
    double angleToBall;
    if (brain->data->ballDetected) {
        angleToBall = brain->data->ball.yawToRobot + robotTheta; 
    } 
    else {
        angleToBall = atan2(brain->data->ball.posToField.y - robotY, brain->data->ball.posToField.x - robotX);
    }
    
    double angleDiff = toPInPI(angleToGoal - angleToBall);
    targetTheta = toPInPI(angleToBall + angleDiff * 0.5); 

    // 각도 오차 계산
    double angleDiffForControl = toPInPI(targetTheta - robotTheta);
    
    // 회전 속도 계산
    double vtheta = angleDiffForControl * 4.0; 
    
    // 안전하게 돌기 위해 최대 회전 속도 제한
    vtheta = cap(vtheta, 1.0, -1.0);

    // 머리 움직임 제어 - scanStartTime, smoothHeadYaw는 헤더에 선언된 멤버 변수 사용
    auto now = std::chrono::steady_clock::now();
    
    double headPitch = 0.0;
    double headYaw = 0.0;
    
    // 6초마다 2초간 고개 돌리기
    double timeSinceScan = std::chrono::duration<double>(now - scanStartTime).count();
    if (timeSinceScan > 6.0) {
        scanStartTime = now;
    }
    
    double targetHeadYaw = 0.0;

    if (timeSinceScan < 2.0) {
        // sin파로 0.8 rad 범위로 2.0초 주기 (왼쪽 1초, 오른쪽 1초)
        targetHeadYaw = 0.8 * sin(2.0 * M_PI * timeSinceScan / 2.0);
    } else {
        // headYaw는 (공 각도 - 로봇의 현재 몸통 각도)
        targetHeadYaw = toPInPI(angleToBall - robotTheta);
        targetHeadYaw = cap(targetHeadYaw, 1.35, -1.35);
    }

    // EMA Smoothing (Low Pass Filter) -> alpha가 클수록 기존 값 유지 성향이 강함 (= 부드러움, 반응 느림)
    // 0.6 정도면 적절한 반응성과 부드러움
    smoothHeadYaw = smoothHeadYaw * 0.6 + targetHeadYaw * 0.4;
    
    // 최종 Check
    headYaw = cap(smoothHeadYaw, 1.35, -1.35);

    brain->client->moveHead(headPitch, headYaw);

    // 최종 속도 명령 전송
    brain->client->setVelocity(vx_robot, vy_robot, vtheta, false, false, false);

    // rerun 로그
    {
        brain->log->setTimeNow();
        // 1. 목표 위치 점

        // 2. 이동 경로 화살표 (로봇 -> 목표)
        brain->log->log("debug/offtheball/path", 
            rerun::Arrows2D::from_vectors({{targetX - robotX, -(targetY - robotY)}})
            .with_origins({{robotX, -robotY}})
            .with_colors(0x00FF00FF)
            .with_labels({"Path"})
        );

        // 3. 디버그 info
        auto endTick = std::chrono::high_resolution_clock::now();
        double duration = std::chrono::duration<double, std::milli>(endTick - startTick).count();
        brain->log->log("debug/offtheball/info", 
            rerun::TextLog(format("Score: %.2f, Time: %.2fms, v(%.2f, %.2f, %.2f)", maxScore, duration, vx_robot, vy_robot, vtheta))
        );
    }

    return NodeStatus::SUCCESS;
}
