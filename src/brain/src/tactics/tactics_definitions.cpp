#include "brain.h"
#include "tactics/tactics_definitions.h"
#include "utils/math.h"
#include <chrono>

using namespace std;

// ==========================================
// 전략 1. 압박 전략 (Pressing)
// ==========================================
PortsList TacticPressing::providedPorts()
{
    return {};
}

NodeStatus TacticPressing::tick()
{    
    // 1. 추적 속도 제한
    brain->tree->setEntry("Strategy.param_chase_speed_limit", 1.0);
    
    // 2. 킥 임계값 낮춤 (기회 보이면 슈팅)
    brain->tree->setEntry("Strategy.param_kick_threshold", 0.3); // 30cm 오차 이내면 킥
    
    // 3. 수비 라인 전진
    brain->tree->setEntry("Strategy.param_defense_line_x", 1.0); // 상대 진영까지 전진 허용

    setStatus(NodeStatus::RUNNING);
    return NodeStatus::RUNNING;
}

// ==========================================
// 전략 2. 수비라인 메이킹 (Line Defense - Striker Mid-Block)
// ==========================================
PortsList TacticLineDefense::providedPorts()
{
    return {
        InputPort<double>("dist_from_goal", 3.0, "수비 라인 거리")
    };
}

NodeStatus TacticLineDefense::tick()
{
    // 경기장 규격
    auto fd = brain->config->fieldDimensions;
    
    // --- 파라미터 계산 로직 (기존 로직 활용) ---
    double ballX = brain->data->ball.posToField.x;
    
    // 1. 기본 라인: 공보다 2m 뒤 (Screening)
    double baseX = ballX - 2.0; 

    // 2. 라인 제한 (Clamping)
    if (baseX < -2.0) {
        baseX = -2.0; 
    }
    if (baseX > 2.0) {
        baseX = 2.0;
    }

    // --- 블랙보드 세팅 ---
    // 계산된 수비 라인 X좌표를 블랙보드에 주입
    brain->tree->setEntry("Strategy.param_defense_line_x", baseX);
    
    // 수비 시에는 안정적인 속도로 이동 (너무 빠르지 않게)
    brain->tree->setEntry("Strategy.param_chase_speed_limit", 0.6);
    
    // 확실할 때만 걷어내기 (킥 임계값 높임)
    brain->tree->setEntry("Strategy.param_kick_threshold", 0.1);

    // 디버깅: 설정된 파라미터 시각화
    brain->log->log("debug/tactics/param_defense_line", 
         rerun::Arrows2D::from_vectors({{0.0, 1.0}}) // 수직선 표시 대용
         .with_origins({{baseX, 0.0}})
         .with_colors(0x0000FFFF)
         .with_labels({"DesiredLine"})
    );

    setStatus(NodeStatus::RUNNING);
    return NodeStatus::RUNNING;
}

// ==========================================
// 전략 3. 템포 컨트롤 (Tempo Control)
// ==========================================
PortsList TacticTempoControl::providedPorts()
{
    return {};
}

NodeStatus TacticTempoControl::tick()
{    
    // 천천히 움직임
    brain->tree->setEntry("Strategy.param_chase_speed_limit", 0.4);
    
    // 공을 오래 소유 (킥 임계값 매우 높게 -> 드리블 유도)
    brain->tree->setEntry("Strategy.param_kick_threshold", 0.05); 
    
    // 수비 라인은 현 위치 유지
    brain->tree->setEntry("Strategy.param_defense_line_x", brain->data->robotPoseToField.x);

    setStatus(NodeStatus::RUNNING);
    return NodeStatus::RUNNING;
}

// ==========================================
// 전략 4. 총공격 (All Out - Strategy: ALL_OUT_ATTACK)
// ==========================================
PortsList TacticAllOut::providedPorts()
{
    return {};
}

NodeStatus TacticAllOut::tick()
{
    // 1. 최대 속도
    brain->tree->setEntry("Strategy.param_chase_speed_limit", 1.2);

    // 2. 킥 임계값 대폭 완화
    brain->tree->setEntry("Strategy.param_kick_threshold", 0.5); 

    // 3. 수비 라인 극단적 전진 -> 상대 골라인(4.5m) 까지 전진 - 사실상 수비 복귀 안함
    brain->tree->setEntry("Strategy.param_defense_line_x", 4.5);

    // 디버깅
    brain->log->logToScreen("tactics", "!!! ALL OUT ATTACK !!!", 0xFF0000FF); // 빨간색

    return NodeStatus::SUCCESS; // SyncActionNode이므로 SUCCESS/FAILURE 반환
}
