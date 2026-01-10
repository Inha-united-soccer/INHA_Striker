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
    bool hasOpponent = false;

    std::vector<int> defenderIndices;
    for (size_t idx = 0; idx < Opponents.size(); idx++) {
        if (Opponents[idx].label != "Opponent") continue;
        hasOpponent = true;
        if (std::abs(Opponents[idx].posToField.x - goalX) < 4.0) {
            defenderIndices.push_back(idx); // 항상 올바른 인덱스
        }
    }

    // Y축을 따라 0.2m 간격으로 후보 지점 탐색
    for (double y = -maxY; y <= maxY; y += 0.1) { 
        double distToDefender = 0.0;
        double normalizer = (defenderIndices.size() > 0 ? defenderIndices.size() : 1.0);

        for (const auto& defenderIndex : defenderIndices) {
            double dist = norm(y - Opponents[defenderIndex].posToField.y, baseX - Opponents[defenderIndex].posToField.x);
            dist = cap(dist, 3.0, 0.0); // 3m 면 충분히 멀다. 그 이상 떨어져 있다고 cost를 더 주진 않을것
            distToDefender += dist;
        }

        distToDefender /= normalizer;
        
        double score = 0.0
                     - (fabs(y) * 0.8) // 중앙 선호 (0.0)이 골대 중앙선
                     + (distToDefender * 1.0) // 수비수 거리가 멀수록 선호
                     // - (fabs(y - lastBestY) * 0.5); 이전 위치 선호
                     - (fabs(y - robotY) * 0.5); 로봇 위치 선호
                     
                    
        // 공을 알고 있을 때만 패스 경로 계산이 의미가 있음 -> 공을 바라보고 있지만 안보일 수도 있기에
        if (brain->data->ballDetected) { 
            Line passPath = {brain->data->ball.posToField.x, brain->data->ball.posToField.y, baseX, y};
            for (const auto& opponent : Opponents) {
                if (opponent.label != "Opponent") continue;
                // 상대방이 패스 경로에 얼마나 가까운지
                double distToPassPath = pointMinDistToLine({opponent.posToField.x, opponent.posToField.y}, passPath);
                
                // 0.5m 이내에 있으면 페널티 부여
                if (distToPassPath < 1.0) {
                    score -= (1.0 - distToPassPath) * 2.0; // 가까울수록 큰 페널티 (최대 2.0)
                }
            }
        }

        if (score > maxScore) {
            maxScore = score;
            bestY = y; // 가장 점수가 높은 y좌표 선택
        }
    } // TODO: if maxScore is below ?.?, it's not proper to stay at baseX.
    // TODO: in this case, we need whole new logic

    lastBestY = bestY;

    // 최종 목표 위치 설정
    double targetX = baseX;
    double targetY = lastBestY;

    // 이동 벡터 계산
    double errX = targetX - robotX; // X축 이동 필요량
    double errY = targetY - robotY; // Y축 이동 필요량

    // 필드 좌표계 계산
    double vX_field = errX * 1.0;
    double vY_field = errY * 1.0;

    // 로봇 좌표계로 변환
    double vx_robot = cos(robotTheta) * vX_field + sin(robotTheta) * vY_field;
    double vy_robot = -sin(robotTheta) * vX_field + cos(robotTheta) * vY_field;

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
    double vtheta = angleDiff * 1.0; 
    
    // 안전하게 돌기 위해 최대 회전 속도 제한
    if (vtheta > 1.0) vtheta = 1.0;
    if (vtheta < -1.0) vtheta = -1.0;

    // 최종 속도 명령 전송
    brain->client->setVelocity(vx_robot, vy_robot, vtheta);

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
