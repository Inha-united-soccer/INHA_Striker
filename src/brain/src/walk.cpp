#include "brain.h"
#include "walk.h"
#include "brain_tree.h"

#include <cstdlib>
#include <ctime>

// BehaviorTree Factory에 Test 노드를 생성하는 함수를 등록하는 역할 -> 코드 양 줄일 수 있음
#define REGISTER_WALK_BUILDER(Name)     \
    factory.registerBuilder<Name>( \
        #Name,                     \
        [brain](const string &name, const NodeConfig &config) { return make_unique<Name>(name, config, brain); });


void RegisterWalkNodes(BT::BehaviorTreeFactory &factory, Brain* brain){
    REGISTER_WALK_BUILDER(SetVelocity) // 속도 제어
    REGISTER_WALK_BUILDER(StepOnSpot) // 제자리걸음
    REGISTER_WALK_BUILDER(GoToPosition) // 목표 위치 이동
}

NodeStatus SetVelocity::tick()
{
    double x, y, theta;
    vector<double> targetVec;
    getInput("x", x);
    getInput("y", y);
    getInput("theta", theta);

    auto res = brain->client->setVelocity(x, y, theta);
    return NodeStatus::SUCCESS;
}

NodeStatus StepOnSpot::tick()
{
    std::srand(std::time(0));
    double vx = (std::rand() / (RAND_MAX / 0.02)) - 0.01;

    auto res = brain->client->setVelocity(vx, 0, 0);
    return NodeStatus::SUCCESS;
}

#include "utils/math.h"

NodeStatus GoToPosition::tick(){
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
    getInput("x", targetx); 
    getInput("y", targety); 
    getInput("theta", targettheta); 
    
    // degree to radian
    targettheta = targettheta * M_PI / 180.0;

    // 본인 위치
    auto gPos = brain->data->robotPoseToField;
    double gx = gPos.x, gy = gPos.y, gtheta = gPos.theta;

    double errorx = targetx - gx;
    double errory = targety - gy;
    double targetdir = atan2(errory, errorx); // 내 위치에서 목표지점을 이은 벡터의 각도
    double errortheta = toPInPI(targetdir - gtheta); 

    double dist = norm(errorx, errory); // 목표지점까지의 거리
    double controlx, controly, controltheta;
    double Kp = 1.0;
    double linearFactor = 1.0 / (1.0 + exp(-6.0 * (dist - 0.5)));

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
        controltheta = toPInPI(targettheta - gtheta) * Kp; // 제자리에선 각도 맞춤
    }

    brain->client->setVelocity(controlx, controly, controltheta, false, false, false);
    return NodeStatus::SUCCESS;
}