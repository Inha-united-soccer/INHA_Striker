"""
Striker 오프더볼 시뮬레이터
src/brain/src/offtheball.cpp 로직 기반
Visualized with Dark Mode and Custom Info Panels
"""

from dataclasses import dataclass
from typing import List
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.colors import LinearSegmentedColormap
import matplotlib.ticker as ticker

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
    
    "center_y_weight": 3.0,      # Y축 중앙 선호도
    "defender_dist_weight": 20.0, # 수비수와의 거리 가중치 (멀수록 좋음)
    "defender_dist_cap": 3.0,    # 수비수 거리 이득 최대치 제한 (3m 이상은 동일 취급)
    
    "hysteresis_x_weight": 3.0,  # 현재 로봇 위치 유지 선호도 (X축)
    "hysteresis_y_weight": 3.0,  # 현재 로봇 위치 유지 선호도 (Y축)
    
    "penalty_weight": 10.0,       # 경로 막힘 패널티 기본값
    "path_margin": 1.5,          # 경로 방해 판단 거리
    
    "opp_memory_sec": 3.0,       # 수비수 기억 시간 (시간에 따라서 신뢰도 감소함)
    
    "pass_penalty_weight": 15.0, # 패스 경로 막힘 감점 가중치
    "shot_penalty_weight": 3.0,  # 슛 경로 막힘 감점 가중치
    "movement_penalty_weight": 30.0, # 이동 경로 막힘 감점 가중치
    "symmetry_weight": 10.0,      # 대칭 위치 선호 가중치
    "ball_dist_weight": 3.0,     # 공과의 거리 선호 가중치

    "forward_weight":3.0,       # 공격 방향(전진) 선호 가중치 x좌표 작아질 수록 가중치
    "base_x_weight": 10.0,        # X축 위치 선호도 (baseX 근처 선호)

    "path_confidence": 0.5,      # 경로 신뢰도 (0~1)
    
    "search_x_margin": 1.9,      # 검색 범위 (x)
    "grid_step": 0.05,            # 그리드 간격
    
    # 시각화 설정
    "figsize": (10, 7),
}

# =========================
# 2. 시나리오 설정 (좌표 튜닝)
# =========================
SCENARIO = {
    "robot": {"x": -3.2, "y": -2.5}, # 로봇 현재 위치
    "ball":  {"x": -1.5, "y": 1.0}, # 공 위치
    "passer": {"x": -1.17, "y": 1.17}, # 패서(DF) 위치 (공 뒤쪽)
    
    "opponents": [
        {"x": -4.0, "y": -0.5}, # 골키퍼
        {"x": -2.3, "y": -1.5}, # 상대 수비수 1 
        {"x": -2.0, "y": 1.5}, # 상대 수비수 2
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
            score -= (params["path_margin"] - dist_pass) * params["pass_penalty_weight"] * cf
            
        # 슛 경로 (목표 -> 골대)
        dist_shot = point_to_segment_distance(opp.pos.x, opp.pos.y, *shot_path)
        if dist_shot < params["path_margin"]:
            score -= (params["path_margin"] - dist_shot) * params["shot_penalty_weight"] * cf
            
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
    # Y축 검색 범위: 필드 전체 폭 (여유 0.5 제거)
    # X축 범위: 상대 진영 전체 (Goal Line ~ Center Line)
    min_x = goal_x
    max_x = 0.0
    
    # Y축 범위: 필드 전체 폭
    max_y = fw / 2.0 
    
    # 그리드 탐색 범위 설정
    xs = np.arange(min_x, max_x + 1e-9, params["grid_step"])
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
    path_x = [start_robot.x]
    path_y = [start_robot.y]
    
    current_robot = Pose2D(start_robot.x, start_robot.y)
    
    for _ in range(steps):
        _, _, _, best_pos, _ = compute_costmap(current_robot, ball, opponents, params)
        target_x, target_y = best_pos
        
        err_x = target_x - current_robot.x
        err_y = target_y - current_robot.y
        
        move_x = (err_x * 1.0) * dt
        move_y = (err_y * 1.0) * dt
        
        current_robot.x += move_x
        current_robot.y += move_y
        
        path_x.append(current_robot.x)
        path_y.append(current_robot.y)
        
        if np.hypot(err_x, err_y) < 0.1:
            break
            
    return path_x, path_y

# =========================
# 6. 시각화 및 실행
# =========================
def visualize():
    # Final Style: White Background, Vivid (Dark) Heatmap
    
    robot = Pose2D(**SCENARIO["robot"])
    ball = Pose2D(**SCENARIO["ball"])
    passer = Pose2D(**SCENARIO["passer"])
    opponents = [Opponent(pos=Pose2D(**opp)) for opp in SCENARIO["opponents"]]
    
    # 1. 초기 상태에서의 Heatmap 및 Target 계산
    X, Y, S, best_pos, best_score = compute_costmap(robot, ball, opponents, PARAMS)
    # bx, by = best_pos
    bx, by = -3.1, 0.2
    
    # 2. 동적 경로 시뮬레이션 실행
    sim_path_x, sim_path_y = simulate_path(robot, ball, opponents, PARAMS)
    
    fig, ax = plt.subplots(figsize=PARAMS["figsize"])
    
    # Background color (White)
    fig.patch.set_facecolor('white')
    ax.set_facecolor('white')
    
    fl = PARAMS["field_length"]
    fw = PARAMS["field_width"]
    hfl = fl / 2.0
    hfw = fw / 2.0
    
    # 경기장 그리기 - Black lines for Light Mode
    line_color = 'black'
    ax.add_patch(plt.Rectangle((-hfl, -hfw), fl, fw, fill=False, edgecolor=line_color, linewidth=2, zorder=6))
    ax.plot([0, 0], [-hfw, hfw], color=line_color, linewidth=1, zorder=6)
    ax.add_patch(plt.Circle((0, 0), PARAMS["circle_radius"], fill=False, edgecolor=line_color, linewidth=1, zorder=6))
    
    # Penalty Areas & Goal Areas
    ax.add_patch(plt.Rectangle((-hfl, -PARAMS["penalty_area_width"]/2), PARAMS["penalty_area_length"], PARAMS["penalty_area_width"], fill=False, edgecolor=line_color, linewidth=1, zorder=6))
    ax.add_patch(plt.Rectangle((-hfl, -PARAMS["goal_area_width"]/2), PARAMS["goal_area_length"], PARAMS["goal_area_width"], fill=False, edgecolor=line_color, linewidth=1, zorder=6))
    ax.add_patch(plt.Rectangle((hfl - PARAMS["penalty_area_length"], -PARAMS["penalty_area_width"]/2), PARAMS["penalty_area_length"], PARAMS["penalty_area_width"], fill=False, edgecolor=line_color, linewidth=1, zorder=6))
    ax.add_patch(plt.Rectangle((hfl - PARAMS["goal_area_length"], -PARAMS["goal_area_width"]/2), PARAMS["goal_area_length"], PARAMS["goal_area_width"], fill=False, edgecolor=line_color, linewidth=1, zorder=6))

    # Goals
    gw = PARAMS["goal_width"]
    ax.add_patch(plt.Rectangle((-hfl - 0.6, -gw/2), 0.6, gw, fill=False, edgecolor=line_color, linewidth=2, zorder=6))
    ax.add_patch(plt.Rectangle((hfl, -gw/2), 0.6, gw, fill=False, edgecolor=line_color, linewidth=2, zorder=6))

    # Custom Colormap (Infrared Style: Black -> Purple -> Orange/Red -> Yellow)
    colors = [
        (0/255, 0/255, 0/255),    # Softer Dark (Dark Blue-Grey)
        (62/255, 15/255, 101/255),  # Brighter Purple
        (216/255, 105/255, 69/255),  # Brighter Orange
        (240/255, 229/255, 115/255)  # Pale Yellow
    ]
    
    cmap_name = 'custom_thermal'
    custom_cmap = LinearSegmentedColormap.from_list(cmap_name, colors, N=256)

    # 히트맵 (zorder=0 so grid/lines show on top, alpha=1.0 for deep colors)
    cm = ax.pcolormesh(X, Y, S, cmap=custom_cmap, shading='auto', alpha=1.0, zorder=0)
    
    # 객체 표시
    # 1. Striker (Robot)
    ax.add_patch(plt.Circle((robot.x, robot.y), 0.2, facecolor='blue', alpha=0.8, zorder=10, edgecolor='black', linewidth=2))
    ax.text(robot.x, robot.y, "ST", color='white', ha='center', va='center', fontweight='bold', zorder=11)
    
    # 2. Pass Source (DF)
    ax.add_patch(plt.Circle((passer.x, passer.y), 0.2, facecolor='blue', alpha=0.8, zorder=9, edgecolor='black', linewidth=2))
    ax.text(passer.x, passer.y, "DF", color='white', ha='center', va='center', fontweight='bold', zorder=10)

    # 3. Our GK (Teammate) - Manual Addition at our goal (Positive X)
    # our_gk_x, our_gk_y = 3.5, 0.0
    # ax.add_patch(plt.Circle((our_gk_x, our_gk_y), 0.2, facecolor='blue', alpha=0.8, zorder=9, edgecolor='black', linewidth=2))
    # ax.text(our_gk_x, our_gk_y, "GK", color='white', ha='center', va='center', fontweight='bold', zorder=10)

    # 4. Opponents
    for i, opp in enumerate(opponents):
        if i == 0:
            label = "GK"
        elif i == 1:
            label = "DF"
        else:
            label = "ST"
        
        ax.add_patch(plt.Circle((opp.pos.x, opp.pos.y), 0.2, facecolor='red', alpha=0.8, zorder=10, edgecolor='black', linewidth=2))
        ax.text(opp.pos.x, opp.pos.y, label, color='white', ha='center', va='center', fontweight='bold', zorder=11)

    # Ball (White Dot)
    ax.plot(ball.x, ball.y, 'o', color='white', markersize=12, label="Ball", zorder=12, markeredgecolor='black')

    # 최적 위치 (Target)
    ax.plot(bx, by, 'x', color='white', markersize=15, markeredgewidth=3, zorder=9, label="Optimal Target")
    
    # Paths
    ax.plot([ball.x, bx], [ball.y, by], '-', color='yellow', lw=2.0, alpha=0.7, label="Pass Lane", zorder=7)
    
    goal_x = -(fl/2.0)
    base_x = goal_x + PARAMS["dist_from_goal"]
    # ax.plot([base_x, goal_x], [by, 0], '--', color='magenta', lw=1.5, alpha=0.7, label="Shot Lane", zorder=7)

    # [동적 시뮬레이션 경로]
    n_points = len(sim_path_x)
    step_visual = 3
    for k in range(0, n_points, int(step_visual)):
        # 빨간 원 그림자 (로봇 크기와 동일하게 0.2 radius)
        # ax.plot(sim_path_x[k], sim_path_y[k], 'ro', markersize=12, alpha=0.3, zorder=8)
        ax.add_patch(plt.Circle((sim_path_x[k], sim_path_y[k]), 0.2, color='blue', alpha=0.3, zorder=8, linewidth=0))
    
    ax.plot(sim_path_x, sim_path_y, '.-', color='blue', lw=2, markersize=4, label="Calculated Path", alpha=0.8, zorder=8)

    # Title
    # ax.set_title("Striker Position Analysis", color='black', fontsize=16, fontweight='bold', pad=20)
    
    # Axis Labels
    ax.set_xlabel("Field X (m)", color='black', fontsize=12)
    ax.set_ylabel("Field Y (m)", color='black', fontsize=12)
    
    ax.xaxis.set_major_locator(ticker.MultipleLocator(1.0))
    ax.yaxis.set_major_locator(ticker.MultipleLocator(1.0))
    ax.grid(True, which='major', color='gray', alpha=0.4, linestyle='--', zorder=5)
    
    # Axis Colors
    ax.tick_params(colors='black')
    for spine in ax.spines.values():
        spine.set_edgecolor('black')

    # Info Box
    # dist_to_goal = np.hypot(bx - goal_x, by - 0)
    # info_text = f"Target Score: {best_score:.1f}\nDist to Goal: {dist_to_goal:.1f}m"
    # props = dict(boxstyle='round', facecolor='white', alpha=1.0, edgecolor='black')
    # ax.text(0.02, 0.95, info_text, transform=ax.transAxes, fontsize=12,
    #         verticalalignment='top', color='black', bbox=props, zorder=20)

    # Legend
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], color='yellow', lw=2.0, linestyle='-', label='Pass Lane'),
        Line2D([0], [0], color='blue', lw=1.5, linestyle='-.', label='Calculated Path'),
        #  Line2D([0], [0], color='magenta', lw=1.5, linestyle='--', label='Shot Lane'),
        # Line2D([0], [0], marker='x', color='w', markerfacecolor='yellow', markeredgecolor='yellow', markersize=10, lw=0, label='Optimal Target'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='blue', markeredgecolor='black', markersize=10, lw=0, label='Team (Blue)'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='red', markeredgecolor='black', markersize=10, lw=0, label='Opponent (Red)'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='white', markeredgecolor='black', markersize=8, lw=0, label='Ball'),
    ]
    # Custom Legend Position (Between Center and Center Right)
    # bbox_to_anchor=(x, y): (0.0, 0.0) is bottom-left, (1.0, 1.0) is top-right
    # (0.75, 0.5) puts the center of the legend at 75% width, 50% height
    leg = ax.legend(handles=legend_elements, loc='center', bbox_to_anchor=(0.75, 0.5), 
                    facecolor='white', edgecolor='black', labelcolor='black', fontsize='15', markerscale=1.5)
    leg.set_zorder(40)
    leg.get_frame().set_alpha(0.9)

    # Colorbar
    cbar = fig.colorbar(cm, ax=ax, orientation='vertical', fraction=0.03, pad=0.02)
    cbar.ax.tick_params(labelsize=12)
    cbar.set_label('Score', fontsize=14, labelpad=10)

    # Limits
    ax.set_xlim(-hfl, hfl)
    ax.set_ylim(-hfw, hfw)
    ax.set_aspect('equal', adjustable='box')

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    visualize()
