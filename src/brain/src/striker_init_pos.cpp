#include "brain.h"
#include "brain_tree.h"
#include "striker_init_pos.h"

#include <cmath>
#include "utils/math.h"

#define REGISTER_STRIKERINITPOS_BUILDER(Name)     \
    factory.registerBuilder<Name>( \
        #Name,                     \
        [brain](const string &name, const NodeConfig &config) { return make_unique<Name>(name, config, brain); });


void RegisterStrikerInitPosNodes(BT::BehaviorTreeFactory &factory, Brain* brain){
    REGISTER_STRIKERINITPOS_BUILDER(StrikerInitPos)
}

NodeStatus StrikerInitPos::tick(){
    double turn_Threshold;
    double stop_Threshold;
    double vxLimit;
    double vyLimit;
    getInput("turn_threshold", turn_Threshold); 
    getInput("stop_threshold", stop_Threshold); 
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);

    // 목표 위치
    double targetx, targety, targettheta;
    getInput("init_pos_x", targetx); 
    getInput("init_pos_y", targety); 
    getInput("init_pos_theta", targettheta); 
    
    // degree to radian
    targettheta = targettheta * M_PI / 180.0;

    // 본인 위치
    auto gPos = brain->data->robotPoseToField;
    double gx = gPos.x, gy = gPos.y, gtheta = gPos.theta;

    double errorx = targetx - gx;
    double errory = targety - gy;
    double targetdir = atan2(errory, errorx); // 내 위치에서 목표지점을 이은 벡터의 각도
    double errortheta = toPInPI(targetdir - gtheta); // 이걸 P제어한다면 목표지점을 쳐다볼것.

    double dist = norm(errorx, errory); // 목표지점까지의 거리
    double controlx, controly, controltheta;
    double Kp = 4.0;
    double linearFactor = 1.0 / (1.0 + exp(-6.0 * (dist - 0.5)));

    // 수정 logic similar to GolieInitPos
    if(dist > turn_Threshold){ // 직진
      controlx = errorx*cos(gtheta) + errory*sin(gtheta);
      controly = -errorx*sin(gtheta) + errory*cos(gtheta);
      controlx *= linearFactor;
      controly *= linearFactor;
      controlx = cap(controlx, vxLimit, -vxLimit*0.5);    
      controly = cap(controly, vyLimit, -vyLimit);
      controltheta = errortheta * Kp;
    }
    else if(dist <= turn_Threshold && dist > stop_Threshold){ // 선회
        // 위치 제어는 유지하면서 회전하여 목표 각도 맞춤
        // 하지만 원본 로직에서는 targettheta (최종 바라볼 방향)을 맞추는 것으로 보임
        // "Controltheta = (targettheta - gtheta) * Kp; // 이러면 gtheta(로봇방향)이 targettheta를 바라봄"
        
        controlx = errorx*cos(gtheta) + errory*sin(gtheta);
        controly = -errorx*sin(gtheta) + errory*cos(gtheta);
        controlx *= linearFactor;
        controly *= linearFactor;
        controlx = cap(controlx, vxLimit, -vxLimit*0.5);    
        controly = cap(controly, vyLimit, -vyLimit);
        
        controltheta = toPInPI(targettheta - gtheta) * Kp; 
    }
    else { // 정지 (dist <= stop_Threshold)
        controlx = 0;
        controly = 0;
        controltheta = 0;
    }

    brain->client->setVelocity(controlx, controly, controltheta, false, false, false);
    return NodeStatus::SUCCESS;
}
