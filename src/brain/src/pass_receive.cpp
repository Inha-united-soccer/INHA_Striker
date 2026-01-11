#include "pass_receive.h"
#include "utils/math.h"

using namespace std;
using namespace BT;

NodeStatus PassReceive::onStart()
{
    return NodeStatus::RUNNING;
}

NodeStatus PassReceive::onRunning()
{
    // pass signal false로 초기화
    bool passSignal = false;
    double targetX = 0.0;
    double targetY = 0.0;

    int myId = brain->config->playerId;
    int partnerIdx = (myId == 1) ? 1 : 2;
    
    if (brain->data->tmStatus[partnerIdx].passSignal)
    {
        passSignal = true;
        targetX = brain->data->tmStatus[partnerIdx].passTargetX;
        targetY = brain->data->tmStatus[partnerIdx].passTargetY;
    }

    // target 좌표를 로봇 좌표로 변환
    double target_rx, target_ry, __;
    transCoord(targetX, targetY, 0, 
               brain->data->robotPoseToField.x, brain->data->robotPoseToField.y, brain->data->robotPoseToField.theta, 
               target_rx, target_ry, __);

    double distToTarget = hypot(target_rx, target_ry);
    
    // 이동 -> 추후 장애물 추가
    double p_gain = 1.0;
    double vx = target_rx * p_gain;
    double vy = target_ry * p_gain;

    // 방향 조절
    if (brain->data->ballDetected) {
        vtheta = brain->data->ball.yawToRobot * 1.5; // 공이 보이면 공쪽을 보면서
    } else {
        double targetDir = atan2(target_ry, target_rx);
        vtheta = targetDir * 1.0; // 공이 안보이면 목표 지점을 보면서
    }

    // 속도
    double maxSpeed = 0.8;
    double speed = sqrt(vx*vx + vy*vy);
    if (speed > maxSpeed) {
        vx = vx / speed * maxSpeed;
        vy = vy / speed * maxSpeed;
    }

    // 타겟위치 도착
    if (distToTarget < 0.2) {
        vx = 0.01; vy = 0.01; // 정지보단 조금씩 움직이도록
        // 공도 왔는지 확인
        if (brain->data->ballDetected && brain->data->ball.range < 0.5) {
            return NodeStatus::SUCCESS;
        }
    }

    brain->set_velocity(vx, vy, vtheta);

    return NodeStatus::RUNNING;
}

void PassReceive::onHalted()
{
}

#define REGISTER_PASSRECEIVE_BUILDER(Name)     \
    factory.registerBuilder<Name>( \
        #Name,                     \
        [brain](const string &name, const NodeConfig &config) { return make_unique<Name>(name, config, brain); });

void RegisterPassReceiveNodes(BT::BehaviorTreeFactory &factory, Brain* brain){
    REGISTER_PASSRECEIVE_BUILDER(PassReceive)
}
