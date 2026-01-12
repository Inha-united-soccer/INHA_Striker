"""
Striker 오프더볼 시뮬레이터
src/brain/src/offtheball.cpp 로직 기반
"""

from dataclasses import dataclass
from typing import List
import numpy as np
import matplotlib.pyplot as plt

# =========================
# 1. 튜닝 파라미터
# =========================

# 경기장 설정
PARAMS = {
    "field_length": 9.0,
    "field_width": 6.0,
    "penalty_dist": 1.5,
    "goal_width": 2.0,
    "circle_radius": 0.75,
    "penalty_area_length": 2.0,
    "penalty_area_width": 5.0,
    "goal_area_length": 1.0,
    "goal_area_width": 3.0,
    
    # 로직 가중치 및 설정
    "dist_from_goal": 2.0,       # 오프더볼 거리
    "base_x_weight": 0.1,        # X축 위치 선호도 (baseX 근처 선호)
    "center_y_weight": 0.6,      # Y축 중앙 선호도
    "defender_dist_weight": 1.0, # 수비수와의 거리 가중치 (멀수록 좋음)
    "defender_dist_cap": 3.0,    # 수비수 거리 이득 최대치 제한 (3m 이상은 동일 취급)
    
    "hysteresis_x_weight": 1.3,  # 현재 로봇 위치 유지 선호도 (X축)
    "hysteresis_y_weight": 1.5,  # 현재 로봇 위치 유지 선호도 (Y축)
    
    "penalty_weight": 10.0,       # 경로 막힘 패널티 기본값
    "path_margin": 1.5,          # 경로 방해 판단 거리
    
    "opp_memory_sec": 5.0,       # 수비수 기억 시간 (시간에 따라서 신뢰도 감소함)
    
    "pass_penalty_weight": 5.0,  # 패스 경로 막힘 감점 가중치
    "shot_penalty_weight": 3.0,  # 슛 경로 막힘 감점 가중치
    "movement_penalty_weight": 30.0, # 이동 경로 막힘 감점 가중치
    "symmetry_weight": 10.0,      # 대칭 위치 선호 가중치
    "ball_dist_weight": 2.0,     # 공과의 거리 선호 가중치
    "forward_weight": 2.0,       # 공격 방향(전진) 선호 가중치 x가 작아질 수록 가중치
    
    "path_confidence": 0.5,      # 경로 신뢰도 (0~1)
    
    "search_x_margin": 1.8,      # 검색 범위 (x)
    "grid_step": 0.1,            # 그리드 간격
    
    # 시각화 설정
    "figsize": (12, 6),
}

# =========================
# 2. 시나리오 설정 (좌표 튜닝)
# =========================
SCENARIO = {
    "robot": {"x": -3.0, "y": 3.0}, # 로봇 현재 위치
    "ball":  {"x": -0.1, "y": 0.0}, # 공 위치
    
    "opponents": [
        {"x": -3.8, "y": -0.5}, # 골키퍼
        {"x": -3.0, "y": 2.0}, # 상대 수비수 1 
        {"x": -2.0, "y": 0.5}, # 상대 수비수 2
    ]
}

# =========================
# 3. 데이터 구조 및 유틸리티
# =========================
@dataclass
class Pose2D:
    x: float
    y: float

@dataclass
class Opponent:
    pos: Pose2D
    last_seen_sec_ago: float = 0.0

def point_to_segment_distance(px, py, ax, ay, bx, by):
    """점 P에서 선분 AB까지의 최단 거리 계산"""
    abx, aby = bx - ax, by - ay
    apx, apy = px - ax, py - ay
    ab2 = abx * abx + aby * aby
    if ab2 < 1e-12: return np.hypot(apx, apy)

    t = (apx * abx + apy * aby) / ab2
    t = max(0.0, min(1.0, t))
    cx, cy = ax + t * abx, ay + t * aby
    return np.hypot(px - cx, py - cy)

def confidence_factor(last_seen, memory_sec):
    return max(0.0, (memory_sec - last_seen) / memory_sec)

# =========================
# 4. 점수 계산 로직 (Core)
# =========================
def compute_striker_score(tx: float, ty: float,
                          robot: Pose2D,
                          ball: Pose2D,
                          opponents: List[Opponent],
                          params: dict) -> float:
    
    fl = params["field_length"]
    goal_x = -(fl / 2.0)
    base_x = goal_x + params["dist_from_goal"]
    
    # [기본 점수] 위치 선호도
    score = 0.0
    score -= abs(tx - base_x) * params["base_x_weight"]
    score -= abs(ty) * params["center_y_weight"]
    score -= abs(tx - robot.x) * params["hysteresis_x_weight"]
    score -= abs(ty - robot.y) * params["hysteresis_y_weight"]
    
    # [수비수 회피] 골대 근처 수비수들과 멀리 떨어질수록 좋음
    defenders = [opp for opp in opponents if abs(opp.pos.x - goal_x) < 4.0]
    
    dist_to_defender = 0.0
    normalizer = max(1.0, float(len(defenders)))
    
    for opp in defenders:
        d = np.hypot(ty - opp.pos.y, tx - opp.pos.x)
        d = min(d, params["defender_dist_cap"])
        dist_to_defender += d
        
    dist_to_defender /= normalizer
    score += dist_to_defender * params["defender_dist_weight"]

    # [대칭 위치 선호] 수비수들의 평균 Y 위치의 반대편을 선호
    if defenders:
        avg_opp_y = sum(d.pos.y for d in defenders) / len(defenders)
        sym_target_y = -avg_opp_y # 대칭 목표 지점 (수비수가 왼쪽에 있으면 오른쪽을 선호)
        
        # 대칭 지점과의 거리 페널티 (가까울수록 이득 -> 멀수록 감점) - 단순히 절대값 차이로 계산하면 됨
        # 중앙에 몰려있으면 중앙(0)을 선호하게 되도록
        score -= abs(ty - sym_target_y) * params["symmetry_weight"]

    # [공 거리 선호] 공과의 X축 거리(깊이)가 2.5m와 가까울수록 선호 - Y축은 자유롭게 움직여서 빈 공간(대칭점)을 찾도록 함
    dist_x_to_ball = abs(tx - ball.x)
    score -= abs(dist_x_to_ball - 2.5) * params["ball_dist_weight"]

    # [공격 방향 선호] 골대 쪽(X가 작을수록)으로 갈수록 이득 - tx는 보통 음수이므로, -tx는 양수가 됨 즉 전진할수록 점수 증가
    score += (-tx) * params["forward_weight"]

    # [경로 패널티] 패스 경로 & 슛 경로가 막히면 감점
    pass_path = (ball.x, ball.y, tx, ty)
    shot_path = (base_x, ty, goal_x, 0.0) # 후보 위치에서 골대 중앙으로의 슛 경로

    for opp in opponents:
        cf = confidence_factor(opp.last_seen_sec_ago, params["opp_memory_sec"])
        if cf <= 0.0: continue
        
        # 패스 경로 (공 -> 목표)
        dist_pass = point_to_segment_distance(opp.pos.x, opp.pos.y, *pass_path)
        if dist_pass < params["path_margin"]:
            score -= (1.0 - dist_pass) * params["penalty_weight"] * cf
            
        # 슛 경로 (목표 -> 골대)
        dist_shot = point_to_segment_distance(opp.pos.x, opp.pos.y, *shot_path)
        if dist_shot < params["path_margin"]:
            score -= (1.0 - dist_shot) * params["penalty_weight"] * cf
            
    # 이동 경로 막힘 cost - 수비수 피해가도록
    dist_robot_target = np.hypot(tx - robot.x, ty - robot.y)
    
    # 로봇과 타겟이 너무 가까우면(0.1m 이내) 이동 경로 체크 불필요
    if dist_robot_target > 0.1:
        for opp in opponents:
            cf = confidence_factor(opp.last_seen_sec_ago, params["opp_memory_sec"])
            if cf <= 0.0: continue
            
            # 벡터 정의
            vec_rt_x = tx - robot.x
            vec_rt_y = ty - robot.y
            vec_ro_x = opp.pos.x - robot.x
            vec_ro_y = opp.pos.y - robot.y
            
            # 내적을 통한 투영 계수 t 계산
            dot_prod = vec_ro_x * vec_rt_x + vec_ro_y * vec_rt_y
            len_sq = vec_rt_x**2 + vec_rt_y**2
            t = dot_prod / len_sq
            
            # 가장 가까운 점 찾기
            if t < 0.0:  # 로봇보다 뒤쪽 (고려 X -> 이미 지나온 길은 아님, 출발 전이니까)
                closest_x, closest_y = robot.x, robot.y
            elif t > 1.0: # 타겟 뒤쪽 (고려 X)
                closest_x, closest_y = tx, ty
            else: # 선분 위
                closest_x = robot.x + t * vec_rt_x
                closest_y = robot.y + t * vec_rt_y
            
            dist_to_path = np.hypot(opp.pos.x - closest_x, opp.pos.y - closest_y)
            
            # 경로 마진보다 가까우면 감점
            if t > 0.0 and t < 1.0 and dist_to_path < params["path_margin"]:
                # 거리가 가까울수록 더 큰 감점
                penalty = (params["path_margin"] - dist_to_path) * params["movement_penalty_weight"] * cf
                score -= penalty

    return score


def compute_costmap(robot: Pose2D, ball: Pose2D, opponents: List[Opponent], params: dict):
    fl = params["field_length"]
    fw = params["field_width"]
    goal_x = -(fl / 2.0)
    base_x = goal_x + params["dist_from_goal"]
    max_y = fw / 2.0 - 0.5
    
    # 그리드 탐색 범위 설정
    xs = np.arange(base_x - params["search_x_margin"], base_x + params["search_x_margin"] + 1e-9, params["grid_step"])
    ys = np.arange(-max_y, max_y + 1e-9, params["grid_step"])
    
    X, Y = np.meshgrid(xs, ys)
    S = np.zeros_like(X)
    
    best_score = -1e9
    best_pos = (base_x, 0.0)
    
    for iy in range(X.shape[0]):
        for ix in range(X.shape[1]):
            tx, ty = X[iy, ix], Y[iy, ix]
            sc = compute_striker_score(tx, ty, robot, ball, opponents, params)
            S[iy, ix] = sc
            
            if sc > best_score:
                best_score = sc
                best_pos = (tx, ty)
                
    return X, Y, S, best_pos, best_score

# =========================
# 5. 동적 경로 시뮬레이션
# =========================
def simulate_path(start_robot: Pose2D, ball: Pose2D, opponents: List[Opponent], params: dict, steps=50, dt=0.2):
    """
    로봇이 매 순간 최적의 위치를 다시 계산하며 이동하는 궤적을 시뮬레이션
    Offtheball 로직에는 Hysteresis(현재 내 위치 선호)가 있어서, 
    로봇이 이동함에 따라 목표점(Cost가 가장 낮은 지점)도 미세하게 변할 수 있음.
    """
    path_x = [start_robot.x]
    path_y = [start_robot.y]
    
    current_robot = Pose2D(start_robot.x, start_robot.y)
    
    for _ in range(steps):
        # 1. 현재 위치 기준 최적 목표 계산
        # (로봇 위치가 바뀌면 Hysteresis 항 때문에 Score Map이 바뀜)
        _, _, _, best_pos, _ = compute_costmap(current_robot, ball, opponents, params)
        target_x, target_y = best_pos
        
        # 2. 이동 벡터 계산 (P제어)
        err_x = target_x - current_robot.x
        err_y = target_y - current_robot.y
        
        # 3. 속도 제한 (offtheball.cpp 로직 근사)
        # 실제 코드는 로봇 좌표계 변환 후 리밋을 걸지만, 여기서는 필드 좌표계에서 단순화하여 적용
        # vx_field = err_x * 1.0
        # vy_field = err_y * 1.0
        
        # 이동 (dt초 동안의 변위)
        # 속도 * 시간 = 거리
        move_x = (err_x * 1.0) * dt
        move_y = (err_y * 1.0) * dt
        
        # 위치 업데이트
        current_robot.x += move_x
        current_robot.y += move_y
        
        path_x.append(current_robot.x)
        path_y.append(current_robot.y)
        
        # 목표에 도달했으면 종료 (대락적)
        if np.hypot(err_x, err_y) < 0.1:
            break
            
    return path_x, path_y

# =========================
# 6. 시각화 및 실행
# =========================
def visualize():
    robot = Pose2D(**SCENARIO["robot"])
    ball = Pose2D(**SCENARIO["ball"])
    opponents = [Opponent(pos=Pose2D(**opp)) for opp in SCENARIO["opponents"]]
    
    # 1. 초기 상태에서의 Heatmap 및 Target 계산
    X, Y, S, best_pos, best_score = compute_costmap(robot, ball, opponents, PARAMS)
    bx, by = best_pos
    
    # 2. 동적 경로 시뮬레이션 실행
    sim_path_x, sim_path_y = simulate_path(robot, ball, opponents, PARAMS)
    
    fig, ax = plt.subplots(figsize=PARAMS["figsize"])
    
    fl = PARAMS["field_length"]
    fw = PARAMS["field_width"]
    hfl = fl / 2.0
    hfw = fw / 2.0
    
    # 경기장 그리기 (FD_INHA 규격: 9x6)
    # Outer Boundary
    ax.add_patch(plt.Rectangle((-hfl, -hfw), fl, fw, fill=False, edgecolor='k', linewidth=2))
    
    # Center Line & Circle
    ax.plot([0, 0], [-hfw, hfw], 'k-', linewidth=1)
    ax.add_patch(plt.Circle((0, 0), PARAMS["circle_radius"], fill=False, edgecolor='k', linewidth=1))
    
    # Penalty Areas & Goal Areas
    # Left Side
    ax.add_patch(plt.Rectangle((-hfl, -PARAMS["penalty_area_width"]/2), PARAMS["penalty_area_length"], PARAMS["penalty_area_width"], fill=False, edgecolor='k', linewidth=1))
    ax.add_patch(plt.Rectangle((-hfl, -PARAMS["goal_area_width"]/2), PARAMS["goal_area_length"], PARAMS["goal_area_width"], fill=False, edgecolor='k', linewidth=1))
    
    # Right Side
    ax.add_patch(plt.Rectangle((hfl - PARAMS["penalty_area_length"], -PARAMS["penalty_area_width"]/2), PARAMS["penalty_area_length"], PARAMS["penalty_area_width"], fill=False, edgecolor='k', linewidth=1))
    ax.add_patch(plt.Rectangle((hfl - PARAMS["goal_area_length"], -PARAMS["goal_area_width"]/2), PARAMS["goal_area_length"], PARAMS["goal_area_width"], fill=False, edgecolor='k', linewidth=1))

    # Goals (Goal Width: 2.0)
    gw = PARAMS["goal_width"]
    # Left Goal
    ax.add_patch(plt.Rectangle((-hfl - 0.6, -gw/2), 0.6, gw, fill=False, edgecolor='k', linewidth=2))
    # Right Goal
    ax.add_patch(plt.Rectangle((hfl, -gw/2), 0.6, gw, fill=False, edgecolor='k', linewidth=2))

    # 히트맵 (초기 위치 기준)
    cm = ax.pcolormesh(X, Y, S, cmap='jet', shading='auto', alpha=0.6)
    plt.colorbar(cm, ax=ax, label="Score (at Start)")
    
    # 객체 표시
    ax.plot(robot.x, robot.y, 'bo', markersize=10, label="Robot (Start)")
    ax.plot(ball.x, ball.y, 'ro', markersize=8, label="Ball")
    
    for i, opp in enumerate(opponents):
        ax.plot(opp.pos.x, opp.pos.y, 'rx', markersize=10, markeredgewidth=2)
        ax.annotate(f"OPP{i}", (opp.pos.x+0.1, opp.pos.y+0.1))

    # 최적 위치 및 경로
    ax.plot(bx, by, 'g*', markersize=15, label=f"Target (Initial)\nScore: {best_score:.1f}")
    ax.plot([ball.x, bx], [ball.y, by], 'g--', lw=1, label="Pass Path")
    
    # [동적 시뮬레이션 경로]
    ax.plot(sim_path_x, sim_path_y, 'b.-', lw=2, markersize=3, label="Actual Trajectory")
    
    goal_x = -(fl/2.0)
    base_x = goal_x + PARAMS["dist_from_goal"]
    ax.plot([base_x, goal_x], [by, 0], 'm--', lw=1, label="Shot Path")

    ax.legend(loc='upper right')
    ax.set_title("Striker Off-The-Ball Simulation")
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    visualize()
