#include "brain.h"
#include "adjust.h"
#include "brain_tree.h"

#include <cstdlib>
#include <ctime>

// BehaviorTree Factory에 Test 노드를 생성하는 함수를 등록하는 역할 -> 코드 양 줄일 수 있음
#define REGISTER_ADJUST_BUILDER(Name)     \
    factory.registerBuilder<Name>( \
        #Name,                     \
        [brain](const string &name, const NodeConfig &config) { return make_unique<Name>(name, config, brain); });


void RegisterAdjustNodes(BT::BehaviorTreeFactory &factory, Brain* brain){
    REGISTER_ADJUST_BUILDER(Adjust)
}

NodeStatus Adjust::tick(){
    auto log = [=](string msg) { 
        brain->log->setTimeNow();
        brain->log->log("debug/adjust5", rerun::TextLog(msg)); 
    };
    log("enter");
    if (!brain->tree->getEntry<bool>("ball_location_known"))
    {
        return NodeStatus::SUCCESS;
    }
    // 승재욱 추가
    // if (brain->tree->getEntry<string>("striker_state") != "adjust") return NodeStatus::SUCCESS;

    double vxLimit, vyLimit, vthetaLimit, range, st_far, st_near, vtheta_factor, NEAR_THRESHOLD;
    getInput("near_threshold", NEAR_THRESHOLD);
    getInput("tangential_speed_far", st_far);
    getInput("tangential_speed_near", st_near);
    getInput("vtheta_factor", vtheta_factor);
    // getInput("turn_threshold", turnThreshold);
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    getInput("vtheta_limit", vthetaLimit);
    getInput("range", range);
    double kickYOffset;
    if(!getInput("kick_y_offset", kickYOffset)) kickYOffset = -0.077;

    if (kickYOffset > 0) {
        if (brain->data->ball.posToRobot.y > 0) kickYOffset = fabs(kickYOffset);
        else kickYOffset = -fabs(kickYOffset);
    }

    log(format("ballX: %.1f ballY: %.1f ballYaw: %.1f", brain->data->ball.posToRobot.x, brain->data->ball.posToRobot.y, brain->data->ball.yawToRobot));
    double NO_TURN_THRESHOLD, TURN_FIRST_THRESHOLD;
    getInput("no_turn_threshold", NO_TURN_THRESHOLD);
    getInput("turn_first_threshold", TURN_FIRST_THRESHOLD);


    double vx = 0, vy = 0, vtheta = 0;
    double kickDir = brain->data->kickDir;
    double dir_rb_f = brain->data->robotBallAngleToField; 
    double ballRange = brain->data->ball.range;

    // 한 발로 차기 위해 공을 로봇 중심보다 옆(kickYOffset)에 두도록 정렬
    // deltaDir 각도 에러 수정
    double targetAngleOffset = atan2(kickYOffset, ballRange);
    double deltaDir = toPInPI(kickDir - dir_rb_f + targetAngleOffset);

    double st = st_far; 
    double R = ballRange; 
    double r = range; 
    double sr = cap(R - r, 0.5, -0.2); // 0.2는 너무 가까워질 때 후진도 가능하도록 -> 게걸음 방지
    // R: 현재 공 거리, r: 목표 거리 (0.6), sr: 앞으로 가는 속도 (R-r)
    log(format("R: %.2f, r: %.2f, sr: %.2f, offset: %.2f, targetAng: %.2f", R, r, sr, kickYOffset, targetAngleOffset));

    log(format("deltaDir = %.1f", deltaDir));
    if (fabs(deltaDir) * R < NEAR_THRESHOLD) {
        log("use near speed");
        st = st_near;
    }

    double theta_robot_f = brain->data->robotPoseToField.theta; // 이동 방향 계산용
    double thetat_r = dir_rb_f + M_PI / 2 * (deltaDir > 0 ? -1.0 : 1.0) - theta_robot_f; // 공을 중심으로 회전하는 방향 각도 이 방향으로 st(회전속도) 만큼 움직여서 공 뒤쪽으로 돌아감
    double thetar_r = dir_rb_f - theta_robot_f;  // 로봇 -> 공 각도 이 방향으로 sr(접근속도) 만큼 움직여서 공이랑 거리 맞춤

    vx = st * cos(thetat_r) + sr * cos(thetar_r); 
    vy = st * sin(thetat_r) + sr * sin(thetar_r); 

    // 오프셋 킥 사용 시, 로봇이 공을 바라보는 것이 아니라(ballYaw=0) 킥 방향(골대)과 평행하게 서야 함(kickDir - robotTheta = Heading Error)
    double headingBias = -targetAngleOffset * 0.3; // 30% 정도 공(몸을 안쪽으로)을 바라보게 보정
    double desiredHeading = kickDir + headingBias; // 목표 헤딩 -> 기본적으로 골대를 봐야 잘맞지만 오른발로 차기 위해 몸을 비틀기 때문에 헤딩편향을 섞어줌
    double headingError = toPInPI(desiredHeading - theta_robot_f);
    vtheta = headingError;
    vtheta *= vtheta_factor; 
    // 회전 제어 조건도 ballYaw(공 방향)가 아닌 headingError(골대 방향 + Bias) 기준
    if (fabs(headingError) < NO_TURN_THRESHOLD) vtheta = 0.; 
    
    // 방향이 많이 틀어졌거나 위치가 많이 벗어났으면 일단 제자리 회전
    if (
        fabs(headingError) > TURN_FIRST_THRESHOLD 
        && fabs(deltaDir) < M_PI / 4
    ) { 
        vx = 0;
        vy = 0;
    }

    vx = cap(vx, vxLimit, -0.);
    vy = cap(vy, vyLimit, -vyLimit);
    vtheta = cap(vtheta, vthetaLimit, -vthetaLimit);
    
    log(format("vx: %.1f vy: %.1f vtheta: %.1f", vx, vy, vtheta));
    brain->client->setVelocity(vx, vy, vtheta);

    // 승재욱 추가
    bool adjustDone = fabs(deltaDir) <= 0.1 && fabs(headingError) <= 0.1 && ballRange < range + 0.1;
    if (adjustDone){
        log("adjust -> kick (ready)");
    }
    log(format("deltaDir = %.1f", deltaDir));

    return NodeStatus::SUCCESS;
}