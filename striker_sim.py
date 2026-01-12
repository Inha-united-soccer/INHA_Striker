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
    
    # 로직 가중치 및 설정
    "dist_from_goal": 2.0,       # 오프더볼 거리
    "base_x_weight": 0.1,        # X축 위치 선호도 (baseX 근처 선호)
    "center_y_weight": 0.6,      # Y축 중앙 선호도
    "defender_dist_weight": 1.0, # 수비수와의 거리 가중치 (멀수록 좋음)
    "defender_dist_cap": 3.0,    # 수비수 거리 이득 최대치 제한 (3m 이상은 동일 취급)
    
    "hysteresis_x_weight": 1.0,  # 현재 로봇 위치 유지 선호도 (X축)
    "hysteresis_y_weight": 1.5,  # 현재 로봇 위치 유지 선호도 (Y축)
    
    "penalty_weight": 5.0,       # 경로 막힘 패널티 기본값
    "path_margin": 1.0,          # 경로 방해 판단 거리
    
    "opp_memory_sec": 5.0,       # 수비수 기억 시간 (시간에 따라서 신뢰도 감소함)
    
    # 탐색 그리드 설정
    "search_x_margin": 1.0,      # baseX 기준 앞뒤 탐색 범위 (+/- 1m)
    "grid_step": 0.1,            # 그리드 간격
    
    # 시각화 설정
    "figsize": (12, 6),
}

# =========================
# 2. 시나리오 설정 (좌표 튜닝)
# =========================
SCENARIO = {
    "robot": {"x": -3.0, "y": 1.0}, # 로봇 현재 위치
    "ball":  {"x": 0.1, "y": 0.0}, # 공 위치
    
    "opponents": [
        {"x": 0.1, "y": 0.5}, # 상대 수비수 1 (일반)
        {"x": -2.0, "y": 1.5}, # 상대 수비수 2 (패스 길목)
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
# 5. 시각화 및 실행
# =========================
def visualize():
    robot = Pose2D(**SCENARIO["robot"])
    ball = Pose2D(**SCENARIO["ball"])
    opponents = [Opponent(pos=Pose2D(**opp)) for opp in SCENARIO["opponents"]]
    
    X, Y, S, best_pos, best_score = compute_costmap(robot, ball, opponents, PARAMS)
    bx, by = best_pos
    
    fig, ax = plt.subplots(figsize=PARAMS["figsize"])
    
    fl = PARAMS["field_length"]
    fw = PARAMS["field_width"]
    hfl = fl / 2.0
    hfw = fw / 2.0
    
    # 경기장 그리기
    ax.set_xlim(-hfl - 1, hfl + 1)
    ax.set_ylim(-hfw - 1, hfw + 1)
    ax.set_aspect("equal")
    ax.plot([-hfl, hfl, hfl, -hfl, -hfl], [-hfw, -hfw, hfw, hfw, -hfw], 'k-', lw=2)
    ax.plot([-(hfl), -(hfl)], [-1, 1], 'k-', lw=2) # 골대
    
    # 히트맵
    cm = ax.pcolormesh(X, Y, S, cmap='jet', shading='auto', alpha=0.6)
    plt.colorbar(cm, ax=ax, label="Score")
    
    # 객체 표시
    ax.plot(robot.x, robot.y, 'bo', markersize=10, label="Robot")
    ax.plot(ball.x, ball.y, 'ro', markersize=8, label="Ball")
    
    for i, opp in enumerate(opponents):
        ax.plot(opp.pos.x, opp.pos.y, 'rx', markersize=10, markeredgewidth=2)
        ax.annotate(f"OPP{i}", (opp.pos.x+0.1, opp.pos.y+0.1))

    # 최적 위치 및 경로
    ax.plot(bx, by, 'g*', markersize=15, label=f"Target\n{best_score:.1f}")
    ax.plot([ball.x, bx], [ball.y, by], 'g--', lw=1, label="Pass Path")
    
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
