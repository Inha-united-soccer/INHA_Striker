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

    // 경기장 규격 및 파라미터 가져오기
    auto fd = brain->config->fieldDimensions;
    
    // 골대 앞에서 얼마나 떨어져 있을지
    double distFromGoal = 2.0; 
    if (!getInput("dist_from_goal", distFromGoal)) {
        distFromGoal = 2.0;
    }
    
    // 골대 중앙 좌표
    double goalX = -(fd.length / 2.0);
    // 골대에서 distFromGoal만큼 떨어진 좌표
    double baseX = goalX + distFromGoal; 

    // 최적의 Y좌표 계산 (경기장 폭의 절반에서 0.5m 안쪽)
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
            defenderIndices.push_back(idx); // 항상 올바른 인덱스
        }
    }

    // Calculate symmetry target Y
    double symTargetY = 0.0;
    if (!defenderIndices.empty()) {
        double totalOppY = 0.0;
        for (int idx : defenderIndices) {
            totalOppY += Opponents[idx].posToField.y;
        }
        symTargetY = -(totalOppY / defenderIndices.size());
    }

    // Y축을 따라 0.2m 간격으로 후보 지점 탐색 (범위 확장: +/- 2.0m)
    for (double x = baseX-2.0; x <= baseX+2.0; x += 0.1) {
        for (double y = -maxY; y <= maxY; y += 0.1) {
            double distToDefender = 0.0;
            double normalizer = (defenderIndices.size() > 0 ? defenderIndices.size() : 1.0);

            for (const auto& defenderIndex : defenderIndices) {
                double dist = norm(y - Opponents[defenderIndex].posToField.y, baseX - Opponents[defenderIndex].posToField.x);
                dist = cap(dist, 3.0, 0.0); // 3m 면 충분히 멀다. 그 이상 떨어져 있다고 cost를 더 주진 않을것
                distToDefender += dist;
            }

            distToDefender /= normalizer;
            
            double score = 0.0;
            score -= (fabs(x - baseX) * 0.0); // Sim: base_x_weight = 0.0
            score -= (fabs(y) * 0.6);         // Sim: center_y_weight = 0.6
            score -= (fabs(x - robotX) * 3.0); // Sim: hysteresis_x_weight = 3.0
            score -= (fabs(y - robotY) * 3.0); // Sim: hysteresis_y_weight = 3.0
            score += (distToDefender * 1.0);   // Sim: defender_dist_weight = 1.0

            // [대칭 위치 선호]
            if (!defenderIndices.empty()) {
                score -= std::abs(y - symTargetY) * 10.0; // Sim: symmetry_weight = 10.0
            }

            // [공 거리 선호] X축 거리(깊이) 2.5m 유지
            // 공을 보고 있지 않아도 ball info는 유효하다고 가정 (추정값)
            double distXToBall = std::abs(x - brain->data->ball.posToField.x);
            score -= std::abs(distXToBall - 2.5) * 1.5; // Sim: ball_dist_weight = 1.5

            // [공격 방향 선호] 전진할수록 이득
            // x가 음수이므로, -x는 양수. 즉 전진할수록 점수 증가
            score += (-x) * 5.2; // Sim: forward_weight = 5.2

            // 공을 알고 있을 때만 패스 경로 계산이 의미가 있음 -> 공을 바라보고 있지만 안보일 수도 있기에
            // 생각해보면 메모리가 필수일 거 같아서 우선 추가만 함 봐보고 아니다 싶으면 지우죠

            Line passPath = {brain->data->ball.posToField.x, brain->data->ball.posToField.y, x, y};
            Line shotPath = {baseX, y, goalX, 0.0}; // 후보 위치에서 골대까지의 경로

            for (const auto& opponent : Opponents) {
                if (opponent.label != "Opponent") continue; // 모든 상대편에 대해?

                // 메모리 기반 신뢰도 계산 -> 5초가 지나면 0이 되어 영향력 없음
                rclcpp::Time now = brain->get_clock()->now();
                double elapsed = (now - opponent.timePoint).seconds(); // 수비수를 마지막으로 본 지 몇 초 지났나
                double confidenceFactor = std::max(0.0, (5.0 - elapsed) / 5.0);  // 시간이 지날수록 신뢰도가 떨어지게

                if (confidenceFactor <= 0.0) continue;

                // 패스 경로 cost 계산 (공이 보일 때만)
                if (brain->data->ballDetected) { 
                    double distToPassPath = pointMinDistToLine({opponent.posToField.x, opponent.posToField.y}, passPath);
                    // Sim uses 'penalty_weight' (10.0) and 'path_margin' (1.5)
                    if (distToPassPath < 1.5) { 
                        score -= (1.5 - distToPassPath) * 10.0 * confidenceFactor;
                    }
                }

                // 골대 슈팅각 cost 계산 (공 안보여도 수행)
                double distToShotPath = pointMinDistToLine({opponent.posToField.x, opponent.posToField.y}, shotPath); 
                // Sim uses 'penalty_weight' (10.0) and 'path_margin' (1.5)
                if (distToShotPath < 1.5) { 
                     score -= (1.5 - distToShotPath) * 10.0 * confidenceFactor; 
                }

                // 이동 경로 cost 계산 - 로봇이 후보 위치로 가는 길이 막히면 감점
                Line movementPath = {robotX, robotY, x, y}; // 현재 로봇 위치에서 후보 위치까지의 이동 경로
                double distRobotTarget = norm(x - robotX, y - robotY);
                if (distRobotTarget > 0.1) { // 0.1m 이상 이동해야 할 때만 체크
                     double distToMovementPath = pointMinDistToLine({opponent.posToField.x, opponent.posToField.y}, movementPath);
                     
                     // Sim params: path_margin = 1.5, movement_penalty_weight = 30.0
                     if (distToMovementPath < 1.5) { 
                         score -= (1.5 - distToMovementPath) * 30.0 * confidenceFactor; 
                     }
                }
            }

            if (score > maxScore) {
                maxScore = score;
                bestX = x;
                bestY = y; // 가장 점수가 높은 y좌표 선택
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
    
    // 오프더볼 상황에서는 공을 주시하는 것이 유리함
    if (brain->data->ballDetected) {
        targetTheta = brain->data->ball.yawToRobot + robotTheta; // 공 각도
    } 
    else {
        targetTheta = atan2(0.0 - robotY, goalX - robotX); // 공 모르면 중앙 보기
    }

    // 각도 오차 계산
    double angleDiff = toPInPI(targetTheta - robotTheta);
    
    // 회전 속도 계산
    double vtheta = angleDiff * 4.0; 
    
    // 안전하게 돌기 위해 최대 회전 속도 제한
    vtheta = cap(vtheta, 1.0, -1.0);

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
        brain->log->log("debug/offtheball/info", 
            rerun::TextLog(format("Score: %.2f, DistDiff: %.2f, v(%.2f, %.2f, %.2f)", maxScore, fabs(bestY - lastBestY), vx_robot, vy_robot, vtheta))
        );
    }

    return NodeStatus::SUCCESS;
}
