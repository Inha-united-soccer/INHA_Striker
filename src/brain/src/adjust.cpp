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

    double turnThreshold, vxLimit, vyLimit, vthetaLimit, range, st_far, st_near, vtheta_factor, NEAR_THRESHOLD;
    getInput("near_threshold", NEAR_THRESHOLD);
    getInput("tangential_speed_far", st_far);
    getInput("tangential_speed_near", st_near);
    getInput("vtheta_factor", vtheta_factor);
    getInput("turn_threshold", turnThreshold);
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    getInput("vtheta_limit", vthetaLimit);
    getInput("range", range);
    double kickYOffset;
    if(!getInput("kick_y_offset", kickYOffset)) kickYOffset = -0.08; // [Offset] 킥 오프셋 파라미터 읽기 (기본값 0.0)

    log(format("ballX: %.1f ballY: %.1f ballYaw: %.1f", brain->data->ball.posToRobot.x, brain->data->ball.posToRobot.y, brain->data->ball.yawToRobot));
    double NO_TURN_THRESHOLD, TURN_FIRST_THRESHOLD;
    getInput("no_turn_threshold", NO_TURN_THRESHOLD);
    getInput("turn_first_threshold", TURN_FIRST_THRESHOLD);


    double vx = 0, vy = 0, vtheta = 0;
    double kickDir = brain->data->kickDir;
    double dir_rb_f = brain->data->robotBallAngleToField; 
    // double deltaDir = toPInPI(kickDir - dir_rb_f);
    double deltaDirVal = toPInPI(kickDir - dir_rb_f);
    double ballRange = brain->data->ball.range;

    // 한 발로 차기 위해 공을 로봇 중심보다 옆(kickYOffset)에 두도록 정렬
    // deltaDir 각도 에러 수정
    double targetAngleOffset = atan2(kickYOffset, ballRange);
    double deltaDir = toPInPI(kickDir - dir_rb_f + targetAngleOffset);

    double ballYaw = brain->data->ball.yawToRobot;
    // double st = cap(fabs(deltaDir), st_far, st_near);
    double st = st_far; 
    double R = ballRange; 
    double r = range; 
    double sr = cap(R - r, 0.5, 0); 
    log(format("R: %.2f, r: %.2f, sr: %.2f, offset: %.2f, targetAng: %.2f", R, r, sr, kickYOffset, targetAngleOffset));

    log(format("deltaDir = %.1f", deltaDir));
    if (fabs(deltaDir) * R < NEAR_THRESHOLD) {
        log("use near speed");
        st = st_near;
        // sr = 0.;
        // vxLimit = 0.1;
    }

    double theta_robot_f = brain->data->robotPoseToField.theta; 
    double thetat_r = dir_rb_f + M_PI / 2 * (deltaDir > 0 ? -1.0 : 1.0) - theta_robot_f; 
    double thetar_r = dir_rb_f - theta_robot_f; 

    vx = st * cos(thetat_r) + sr * cos(thetar_r); 
    vy = st * sin(thetat_r) + sr * sin(thetar_r); 
    // vtheta = toPInPI(ballYaw + st / R * (deltaDir > 0 ? 1.0 : -1.0)); 
    // vtheta = ballYaw;
    
    // 오프셋 킥 사용 시, 로봇이 공을 바라보는 것이 아니라(ballYaw=0), 
    // 킥 방향(골대)과 평행하게 서야 함. (kickDir - robotTheta = Heading Error)
    double headingError = toPInPI(kickDir - theta_robot_f);
    vtheta = headingError;
    vtheta *= vtheta_factor; 
    // if (fabs(ballYaw) < NO_TURN_THRESHOLD) vtheta = 0.;
    // 회전 제어 조건도 ballYaw(공 방향)가 아닌 headingError(골대 방향) 기준으로 변경
    if (fabs(headingError) < NO_TURN_THRESHOLD) vtheta = 0.; 
    
    // 방향이 많이 틀어졌거나 위치가 많이 벗어났으면 일단 제자리 회전
    if (
        // fabs(ballYaw) > TURN_FIRST_THRESHOLD 
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
    // bool adjustDone = fabs(deltaDir) <= 0.1 && fabs(ballYaw) <= 0.1 && ballRange < range + 0.1;
    bool adjustDone = fabs(deltaDir) <= 0.1 && fabs(headingError) <= 0.1 && ballRange < range + 0.1;
    if (adjustDone){
        // brain->tree->setEntry("striker_state", "kick");
        log("adjust -> kick (ready)");
    }
    log(format("deltaDir = %.1f", deltaDir));

    return NodeStatus::SUCCESS;
}