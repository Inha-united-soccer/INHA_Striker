#include "brain.h"
#include "brain_tree.h"
#include "kick.h"

// BehaviorTree Factory에 Test 노드를 생성하는 함수를 등록하는 역할 -> 코드 양 줄일 수 있음
#define REGISTER_KICK_BUILDER(Name)     \
    factory.registerBuilder<Name>( \
        #Name,                     \
        [brain](const string &name, const NodeConfig &config) { return make_unique<Name>(name, config, brain); });


void RegisterKickNodes(BT::BehaviorTreeFactory &factory, Brain* brain){
    REGISTER_KICK_BUILDER(CalcKickDir)
    REGISTER_KICK_BUILDER(CalcKickDirWithGoalkeeper)
    REGISTER_KICK_BUILDER(CalcPassDir)
    REGISTER_KICK_BUILDER(Kick)
}

// 해당 노드는 반코트용으로 짰음 -> 풀코트 용으로 나중에 수정 필요함 
// 제안 : 반코트, 풀코트 구분하도록 HalfCalcKickDir, FullCalcKickDir로 구분할 수도 있으니 알아서 잘 활용하길 바람
// 추가해야 될 것 -> 수비 상황 판단
NodeStatus CalcKickDir::tick(){
    // 좌·우 골포스트 각도 차가 이 값보다 작으면 → 골대가 “좁게 보인다” = 정면 슛 각이 안 나온다
    double crossThreshold;
    getInput("cross_threshold", crossThreshold);

    string lastKickType = brain->data->kickType;
    if (lastKickType == "cross") crossThreshold += 0.1; // 이전 type이 cross였다면 좀 더 threshold를 높임으로써 cross를 유지할 수 있도록 함

    auto gpAngles = brain->getGoalPostAngles(0.0); // 공과 골대 각도 계산 -> 마진이 0으로 돼있음
    auto thetal = gpAngles[0]; auto thetar = gpAngles[1]; // 공 기준 좌우 골대 각도
    auto bPos = brain->data->ball.posToField; // 공 위치
    auto fd = brain->config->fieldDimensions; // 필드 정보
    auto color = 0xFFFFFFFF;

    // 풀코트용 슛 방향 계산
    // if (thetal - thetar < crossThreshold && brain->data->ball.posToField.x > fd.circleRadius) {
    //     brain->data->kickType = "cross";
    //     color = 0xFF00FFFF;
    //     brain->data->kickDir = atan2(
    //         - bPos.y,
    //         fd.length/2 - fd.penaltyDist/2 - bPos.x
    //     );
    // }
    // else if (brain->isDefensing()) {
    //     brain->data->kickType = "block";
    //     color = 0xFFFF00FF;
    //     brain->data->kickDir = atan2(
    //         bPos.y,
    //         bPos.x + fd.length/2
    //     );
    // } else { 
    //     brain->data->kickType = "shoot";
    //     color = 0x00FF00FF;
    //     brain->data->kickDir = atan2(
    //         - bPos.y,
    //         fd.length/2 - bPos.x
    //     );
    //     if (brain->data->ball.posToField.x > brain->config->fieldDimensions.length / 2) brain->data->kickDir = 0; 
    // }

    // 반코트용 슛 방향 계산 ( 상대 골대가 음수라고 가정)
    // 추가해야 될 것 -> 수비 상황 판단
    // 그리고 항상 왼쪽이 양수일까?도 고민해보기
    if (thetal - thetar < crossThreshold) {
        brain->data->kickType = "cross";
        color = 0xFF00FFFF;
        // atan2( 목표Y - 공Y , 목표X - 공X )
        // 반코트는 우리팀 진영을 상대팀으로 인식해야하므로 - 값이 들어감
        brain->data->kickDir = atan2( 
            0 - bPos.y, 
            - (fd.length/2 - fd.penaltyDist/2) - bPos.x); // 앞에 - 붙였음
    }
    else { 
        brain->data->kickType = "shoot";
        color = 0x00FF00FF;
        brain->data->kickDir = atan2(
            0 - bPos.y,
            - fd.length/2 - bPos.x 
        );
        if (brain->data->ball.posToField.x < - (brain->config->fieldDimensions.length / 2)) brain->data->kickDir = M_PI; 
    }

    brain->log->setTimeNow();
    brain->log->log(
        "field/kick_dir",
        rerun::Arrows2D::from_vectors({{10 * cos(brain->data->kickDir), -10 * sin(brain->data->kickDir)}})
            .with_origins({{brain->data->ball.posToField.x, -brain->data->ball.posToField.y}})
            .with_colors({color})
            .with_radii(0.01)
            .with_draw_order(31)
    );

    return NodeStatus::SUCCESS;
}

NodeStatus CalcKickDirWithGoalkeeper::tick(){
    // cross 역할은 수행하지 않을 것이므르 crossThreshold 삭제
    double goalkeeperMargin;
    getInput("goalkeeper_margin", goalkeeperMargin);

    double goalAreaBuffer = 0.5; // Default value
    getInput("goal_area_buffer", goalAreaBuffer);

    auto bPos = brain->data->ball.posToField;
    auto fd = brain->config->fieldDimensions;
    
    string strategy = "shoot";
    auto color = 0x00FF00FF;
    
    // CalcKickDir과 동일한 로직으로 중앙을 default로
    double goalX = - (brain->config->fieldDimensions.length / 2); 
    
    // 중앙선 넘을 때 튀는 현상 제거 -> 무조건 중앙(0.0)을 기본 목표로
    _targetGoalY = 0.0; 

    // 공이 골대 라인보다 뒤에 있는 경우(또는 매우 근접), atan2가 뒤집히지 않도록 X 차이를 최소값으로 고정
    // goalX - bPos.x 가 양수(뒤쪽)가 되면 방향이 반대(우리 진영)로 튐. 이를 음수로 강제함
    double dx = goalX - bPos.x;
    if (dx > -0.5) dx = -0.5; // 최소 50cm 앞을 보는 각도로 고정

    double targetKickDir = atan2(
        _targetGoalY - bPos.y,
        dx
    );
    
    // 기본적으로 중앙을 목표하지만(킥의 부정확함) 그 경로에 opponent가 있다면 회피 로직
    vector<GameObject> obstacles = brain->data->getObstacles();
    vector<GameObject> goalkeepers;
    
    // 만약 장애물이 골영역 근처에 있다면 (범위 약간 여유있게)
    for(const auto& obs : obstacles){
        if(obs.posToField.x < goalX + fd.goalAreaLength + goalkeeperMargin + goalAreaBuffer){ 
            goalkeepers.push_back(obs);
        }
    }

    if(!goalkeepers.empty()){
        auto gk = goalkeepers[0];
        double distToGK = norm(gk.posToField.x - bPos.x, gk.posToField.y - bPos.y);
        double angleToGK = atan2(gk.posToField.y - bPos.y, gk.posToField.x - bPos.x);
        double angularWidth = atan2(goalkeeperMargin, distToGK);
        
        double diff = toPInPI(angleToGK - targetKickDir);
        
        // 이미 피하고 있는 방향이 있다면(_lastGapSide != 0), 그 방향을 유지하기 위한 조건을 완화해줌
        double currentAngularWidth = angularWidth;
        if (_lastGapSide != 0) currentAngularWidth *= 0.8; // 한 번 피하기 시작했으면 유지하려는 성질 (Threshold 낮춤)

        // GK가 킥 경로를 막고 있다면
        if(fabs(diff) < currentAngularWidth){
            double angleLeftPost = atan2(fd.goalWidth/2 - bPos.y, dx);
            double angleRightPost = atan2(-fd.goalWidth/2 - bPos.y, dx);
            double angleToGoalCenter = atan2(0 - bPos.y, dx);
            
            // GK가 중앙보다 왼쪽에 있나 오른쪽에 있나?
            double gkDiff = angleToGK - angleToGoalCenter;
            gkDiff = atan2(sin(gkDiff), cos(gkDiff));

            string gapChoice = "Center";
            
            // 기존 선택 유지 경향성 추가
            bool aimLeft = (gkDiff > 0);

            double distToLeftPost = fabs(toPInPI(angleToGK - angleLeftPost));
            double distToRightPost = fabs(toPInPI(angleToGK - angleRightPost));
            
            
            if (_lastGapSide == 1) distToLeftPost += 0.2; 
            else if (_lastGapSide == -1) distToRightPost += 0.2;

            if (distToLeftPost > distToRightPost) {
                // 왼쪽 포스트가 GK로부터 더 멀다 -> 왼쪽 틈으로 슛
                targetKickDir = angleLeftPost + (angleToGoalCenter - angleLeftPost) * 0.4; //1.0이 중앙, 0.0이 기둥
                gapChoice = "Left Gap";
                _lastGapSide = 1;
            } else {
                // 오른쪽 틈으로 슛
                targetKickDir = angleRightPost + (angleToGoalCenter - angleRightPost) * 0.4; //1.0이 중앙, 0.0이 기둥
                gapChoice = "Right Gap";
                _lastGapSide = -1;
            }
            brain->log->logToScreen("debug/KickDir", format("GK Blocking! Aiming: %s", gapChoice.c_str()), 0xFF0000FF);
        } else {
            _lastGapSide = 0; // 안 막혀있으면 리셋
        }
    } else {
        _lastGapSide = 0;
    }

    brain->data->kickType = strategy;
    
    // 필터 강도 조절 (0.5 -> 0.1) : 갑작스러운 튐 방지
    // chase_threshold 밖에서는 필터링을 강하게, 안에서는 약하게? 
    // 일단 전체적으로 부드럽게
    double prevKickDir = brain->data->kickDir; 
    
    // 골키퍼가 움직이는 각도 어느정도 무시
    double diff = toPInPI(targetKickDir - prevKickDir);
    if(fabs(diff) < 0.05) diff = 0.0; 

    brain->data->kickDir = prevKickDir + diff * 0.3; // 회피 반응속도 0.1은 거의 변화가 없어버림
    brain->data->kickDir = toPInPI(brain->data->kickDir);
    
    brain->log->setTimeNow();
    brain->log->log(
        "field/kick_dir_gk",
        rerun::Arrows2D::from_vectors({{10 * cos(brain->data->kickDir), -10 * sin(brain->data->kickDir)}})
            .with_origins({{brain->data->ball.posToField.x, -brain->data->ball.posToField.y}})
            .with_colors({color}) 
            .with_radii(0.01) 
            .with_labels({"KickDir"})
            .with_draw_order(32)
    );

    return NodeStatus::SUCCESS;
}

NodeStatus CalcPassDir::tick(){
    double passThreshold;
    getInput("pass_threshold", passThreshold);

    auto bPos = brain->data->ball.posToField; // 공 위치
    int bestTeammateIdx = -1;
    double minDist = 9999.0;

    // 가장 가까운(혹은 적절한) 팀원 찾기
    for(int i=0; i<HL_MAX_NUM_PLAYERS; i++){
        // 나 자신 제외, 살아있는 팀원 확인
        if(i + 1 == brain->config->playerId) continue;
        if(!brain->data->tmStatus[i].isAlive) continue; 
        
        // 팀원 위치
        auto tmPos = brain->data->tmStatus[i].robotPoseToField;
        
        // 거리 계산
        double dist = norm(bPos.x - tmPos.x, bPos.y - tmPos.y);

        // 유효 거리 내에 있고, 가장 가까운 팀원 선택 (단순 거리 기준)
        if(dist < passThreshold && dist < minDist){
            minDist = dist;
            bestTeammateIdx = i;
        }
    }

    if(bestTeammateIdx != -1){
        brain->data->kickType = "pass"; // 킥 타입 설정
        auto tmPos = brain->data->tmStatus[bestTeammateIdx].robotPoseToField;
        
        // 공에서 팀원 방향으로 킥 방향 설정
        brain->data->kickDir = atan2(tmPos.y - bPos.y, tmPos.x - bPos.x);
        
        brain->log->logToScreen("debug/Pass", format("Passing to TM %d at Dist %.2f", bestTeammateIdx+1, minDist), 0x00FF00FF);
    } else {
        // 줄 사람 없으면 그냥 슛 (기존 로직 fallback)
        brain->data->kickType = "shoot"; 
        brain->data->kickDir = 0.0; // 정면
        brain->log->logToScreen("debug/Pass", "No Teammate found, fallback to shoot", 0xFF0000FF);
    }

    // 시각화
    brain->log->setTimeNow();
    brain->log->log(
        "field/pass_dir",
        rerun::Arrows2D::from_vectors({{10 * cos(brain->data->kickDir), -10 * sin(brain->data->kickDir)}})
            .with_origins({{brain->data->ball.posToField.x, -brain->data->ball.posToField.y}})
            .with_colors({0x00FFFFFF}) // Cyan color for pass
            .with_radii(0.01)
    );

    return NodeStatus::SUCCESS;
}

// 승재욱 - 직접 만든 Kick
// 반코트용으로 부호 및 부등호 바뀐 부분 존재 -> 풀코트로 수정 필요
tuple<double, double, double> Kick::_calcSpeed() {
    double vx, vy, msecKick;


    double vxLimit, vyLimit;
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    int minMSecKick;
    getInput("min_msec_kick", minMSecKick);
    double vxFactor = brain->config->vxFactor;   
    double yawOffset = brain->config->yawOffset; 


    double kickYOffset;
    getInput("kick_y_offset", kickYOffset);

    // [중요 수정] 이제 Offset을 고려해서 목표 Yaw를 설정함
    // 단순 yawToRobot이 0이 되도록 하는게 아니라, Offset 위치로 공이 오도록 함
    double targetYaw = atan2(kickYOffset, brain->data->ball.range);
    double errorYaw = brain->data->ball.yawToRobot - targetYaw;

    // errorYaw를 줄이는 방향으로 ty 생성
    double adjustedYaw = errorYaw + yawOffset; 
    double tx = cos(adjustedYaw) * brain->data->ball.range; 
    double ty = sin(adjustedYaw) * brain->data->ball.range;

    if (fabs(ty) < 0.01 && fabs(adjustedYaw) < 0.01){ 
        vx = vxLimit;
        vy = 0.0;
    }
    else{ 
        vy = ty > 0 ? vyLimit : -vyLimit;
        vx = vy / ty * tx * vxFactor;
        if (fabs(vx) > vxLimit){
            vy *= vxLimit / vx;
            vx = vxLimit;
        }
    }

    double speed = norm(vx, vy);
    msecKick = speed > 1e-5 ? minMSecKick + static_cast<int>(brain->data->ball.range / speed * 1000) : minMSecKick;
    
    return make_tuple(vx, vy, msecKick);
}

NodeStatus Kick::onStart(){

    // _while로 제어되므로 별도의 state check 불필요
    // if(brain->tree->getEntry<string>("striker_state") != "kick") return NodeStatus::SUCCESS;

    _minRange = brain->data->ball.range;
    _speed = 0.5;
    _startTime = brain->get_clock()->now();

    // 장애물 회피 로직
    bool avoidPushing;
    double kickAoSafeDist;
    brain->get_parameter("obstacle_avoidance.avoid_during_kick", avoidPushing);
    brain->get_parameter("obstacle_avoidance.kick_ao_safe_dist", kickAoSafeDist);
    string role = brain->tree->getEntry<string>("player_role");
    
    if (
        avoidPushing
        && (role != "goal_keeper")
        && brain->data->robotPoseToField.x > -(brain->config->fieldDimensions.length / 2 - brain->config->fieldDimensions.goalAreaLength) // 음수 및 부등호 방향 변경 -> 반코트용
        && brain->distToObstacle(brain->data->ball.yawToRobot) < kickAoSafeDist
    ) {
        brain->client->setVelocity(-0.1, 0, 0);
        
        return NodeStatus::SUCCESS;
    }

    getInput("msecs_stablize", _msecs_stablize);
    if (_msecs_stablize > 0) {
        _state = "stabilize";
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::RUNNING;
    }

    _state = "kick";
    double angle = brain->data->ball.yawToRobot;
    brain->client->crabWalk(angle, _speed);
    
    return NodeStatus::RUNNING;
}

NodeStatus Kick::onRunning(){
    auto log = [=](string msg) {
        brain->log->setTimeNow();
        brain->log->log("debug/Kick", rerun::TextLog(msg));
    };
    string kickType = getInput<string>("kick_type").value();
    brain->log->logToScreen("debug/Action", "Action: " + kickType, 0x00FF00FF);

    if (_state == "stabilize") {
        if (brain->msecsSince(_startTime) > _msecs_stablize) {
            _state = "kick";
            _startTime = brain->get_clock()->now();
            log("stabilize done");
        } else {
            brain->client->setVelocity(0, 0, 0);
            return NodeStatus::RUNNING;
        }
    }

    // if(brain->tree->getEntry<string>("striker_state") != "kick") return NodeStatus::SUCCESS;

    bool enableAbort;
    brain->get_parameter("strategy.abort_kick_when_ball_moved", enableAbort);
    auto ballRange = brain->data->ball.range;
    const double MOVE_RANGE_THRESHOLD = 0.3;
    const double BALL_LOST_THRESHOLD = 1000;  
    
    if (
        enableAbort 
        && (
            (brain->data->ballDetected && ballRange - _minRange > MOVE_RANGE_THRESHOLD) 
            || brain->msecsSince(brain->data->ball.timePoint) > BALL_LOST_THRESHOLD 
        )
    ) {
        log("ball moved, abort kick");
        // brain->tree->setEntry("striker_state", "chase"); // 상태 변경은 Decide 노드에 맡김
        
        return NodeStatus::SUCCESS;
    }

    if (ballRange < _minRange) _minRange = ballRange;    

    bool avoidPushing;
    brain->get_parameter("obstacle_avoidance.avoid_during_kick", avoidPushing);
    double kickAoSafeDist;
    brain->get_parameter("obstacle_avoidance.kick_ao_safe_dist", kickAoSafeDist);
    
    if (
        avoidPushing
        && brain->data->robotPoseToField.x > -(brain->config->fieldDimensions.length / 2 - brain->config->fieldDimensions.goalAreaLength) // 음수 및 부등호 방향 변경 -> 반코트용
        && brain->distToObstacle(brain->data->ball.yawToRobot) < kickAoSafeDist
    ) {
        brain->client->setVelocity(-0.1, 0, 0);
        
        return NodeStatus::SUCCESS;
    }

    // 시간 체크 및 종료 처리
    double msecs = getInput<double>("min_msec_kick").value();
    double speedLimit = getInput<double>("speed_limit").value(); // speed 변수명 겹침 주의해서 speedLimit으로 변경 권장하나 원본 유지
    
    // 원본 로직: 시간에 거리/속도 항을 더함
    msecs = msecs + brain->data->ball.range / speedLimit * 1000;
    
    if (brain->msecsSince(_startTime) > msecs) { 
        brain->client->setVelocity(0, 0, 0);

        // 승재욱 추가
        // brain->tree->setEntry("striker_state", "chase");
        
        return NodeStatus::SUCCESS;
    }

    // [원본 그대로] 가속 로직 (점점 빨라짐)
    // if (brain->data->ballDetected) { 
    //     double angle = brain->data->ball.yawToRobot;
    //     // _speed는 멤버 변수
    //     _speed += 0.1; 
        
    //     // 입력받은 제한 속도와 비교
    //     double currentCmdSpeed = min(speedLimit, _speed);
    //     brain->client->crabWalk(angle, currentCmdSpeed);
    // }
    
     // 승재욱 추가: _calcSpeed 활용하도록 변경
    if(brain->data->ballDetected){
         auto [vx, vy, _] = _calcSpeed();
         
         // 공을 보지 말고, 골대(KickDir)를 봐야 함 -> Bias 적용 (Adjust와 동일)
         double kickYOffset;
         getInput("kick_y_offset", kickYOffset);
         double targetAngleOffset = atan2(kickYOffset, brain->data->ball.range); // Kick::_calcSpeed의 targetYaw와 유사
         double headingBias = -targetAngleOffset * 0.3; // 0.3은 공기준 -> 공을 차기 위한 각도와 현재 로봇 각도 차이를 얼마나 보정할지 : 골대와 볼 기준 점 정하기
         double desiredHeading = brain->data->kickDir + headingBias; // 몸통이 바라볼 최종 각도
         
         double headingError = toPInPI(desiredHeading - brain->data->robotPoseToField.theta);
         double vtheta = headingError * 1.5; // P-gain 1.5
         
         brain->client->setVelocity(vx, vy, vtheta);
    }

    return NodeStatus::RUNNING;
}

void Kick::onHalted(){
    // [원본]
    // brain->tree->setEntry("striker_state", "chase");
    _startTime -= rclcpp::Duration(100, 0);
}
