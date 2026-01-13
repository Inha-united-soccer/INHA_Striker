
import math
import numpy as np
from dataclasses import dataclass

@dataclass
class Pose2D:
    x: float
    y: float

@dataclass
class Teammate:
    player_id: int
    pos: Pose2D
    is_alive: bool = True
    label: str = "Teammate"

@dataclass
class Opponent:
    pos: Pose2D
    last_seen_sec_ago: float = 0.0
    label: str = "Opponent"

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def point_to_segment_distance(px, py, ax, ay, bx, by):
    abx, aby = bx - ax, by - ay
    apx, apy = px - ax, py - ay
    ab2 = abx * abx + aby * aby
    if ab2 < 1e-12: return np.hypot(apx, apy)
    t = (apx * abx + apy * aby) / ab2
    t = max(0.0, min(1.0, t))
    cx, cy = ax + t * abx, ay + t * aby
    return np.hypot(px - cx, py - cy)

def confidence_factor(last_seen_sec_ago, memory_sec):
    if last_seen_sec_ago < 3.0: return 1.0
    return max(0.0, (memory_sec - last_seen_sec_ago) / (memory_sec - 3.0)) if memory_sec > 3.0 else 0.0

def move_towards(cur, target, speed, dt):
    dx, dy = target.x - cur.x, target.y - cur.y
    d = np.hypot(dx, dy)
    if d < 1e-9: return Pose2D(cur.x, cur.y)
    step = speed * dt
    if step >= d: return Pose2D(target.x, target.y)
    ux, uy = dx/d, dy/d
    return Pose2D(cur.x + ux*step, cur.y + uy*step)

def select_best_teammate(ball, teammates, my_player_id, params):
    best = None
    best_score = -1e18
    for tm in teammates:
        if tm.player_id == my_player_id: continue
        if not tm.is_alive: continue
        dist = np.hypot(ball.x - tm.pos.x, ball.y - tm.pos.y)
        if dist <= params["min_pass_threshold"] or dist >= params["max_pass_threshold"]: continue
        score = -(params["tm_select_w_dist"] * dist) - (params["tm_select_w_x"] * tm.pos.x)
        if score > best_score:
            best_score = score
            best = tm
    return best

def compute_pass_costmap(ball, tm, opponents, params):
    hlx = params["field_half_length"]; hly = params["field_half_width"]
    Rx = params["grid_half_xrange"]; Ry = params["grid_half_yrange"]; step = params["grid_step"]
    xs = np.arange(tm.x - Rx, tm.x + Rx + 1e-9, step)
    ys = np.arange(tm.y - Ry, tm.y + Ry + 1e-9, step)
    X, Y = np.meshgrid(xs, ys)
    best_score = -1e18
    best_tx, best_ty = float("nan"), float("nan")
    
    # Vectorization optimized for standard numpy usage within reason
    # To keep identical logic to Pygame version, we iterate or use careful broadcasting
    # Let's keep loop for safety to ensure identical matching logic with previously tested code
    for iy in range(Y.shape[0]):
        for ix in range(X.shape[1]):
            tx, ty = float(X[iy, ix]), float(Y[iy, ix])
            if abs(tx) > hlx or abs(ty) > hly: continue
            pass_dist = np.hypot(tx - ball.x, ty - ball.y)
            if pass_dist < params["min_pass_threshold"] or pass_dist > params["max_pass_threshold"]: continue
            
            score = (params["base_score"] - (abs(tx - tm.x) * params["w_abs_dx"]) - (abs(ty - tm.y) * params["w_abs_dy"]) - (tx * params["w_x"]) - (abs(ty) * params["w_y"]))
            ax, ay = ball.x, ball.y; bx, by = tx, ty; margin = params["opp_path_margin"]
            for opp in opponents:
                if opp.label != "Opponent": continue
                cf = confidence_factor(opp.last_seen_sec_ago, params["opp_memory_sec"])
                if cf <= 0.0: continue
                d = point_to_segment_distance(opp.pos.x, opp.pos.y, ax, ay, bx, by)
                if d < margin: score -= (1.0 - d/margin) * params["opp_penalty"] * cf
            if score > best_score:
                best_score = score; best_tx, best_ty = tx, ty
    return (best_tx, best_ty, best_score)

def compute_striker_costmap(robot, ball, opponents, params):
    fl = params["field_length"]; goal_x = -(fl / 2.0); base_x = goal_x + params["dist_from_goal"]
    max_y = params["field_width"] / 2.0 - 0.5
    xs = np.arange(base_x - params["search_x_margin"], base_x + params["search_x_margin"] + 1e-9, params["grid_step"])
    ys = np.arange(-max_y, max_y + 1e-9, params["grid_step"])
    X, Y = np.meshgrid(xs, ys)
    best_score = -1e9
    best_pos = (base_x, 0.0)
    
    for iy in range(X.shape[0]):
        for ix in range(X.shape[1]):
            tx, ty = float(X[iy, ix]), float(Y[iy, ix])
            score = 0.0
            score -= abs(tx - base_x) * params["base_x_weight"]
            score -= abs(ty) * params["center_y_weight"]
            score -= abs(tx - robot.x) * params["hysteresis_x_weight"]
            score -= abs(ty - robot.y) * params["hysteresis_y_weight"]
            
            # Defender Dist & Symmetry
            dist_to_defender = 0.0; defenders_in_box = 0
            defender_positions = []
            for opp in opponents:
                 if abs(opp.pos.x - goal_x) < 4.0 and opp.label == "Opponent":
                     defender_positions.append(opp.pos)
                     d = np.hypot(ty - opp.pos.y, tx - opp.pos.x)
                     d = min(d, params["defender_dist_cap"])
                     dist_to_defender += d
                     defenders_in_box += 1
            
            if defenders_in_box > 0: 
                dist_to_defender /= defenders_in_box
            else:
                 # Standardize normalizer if 0 defenders? C++ uses 1.0 logic implicitly by size check
                 pass
                 
            score += dist_to_defender * params["defender_dist_weight"]
            
            # Symmetry (New)
            if defenders_in_box > 0:
                total_opp_y = sum(p.y for p in defender_positions)
                sym_target_y = -(total_opp_y / defenders_in_box)
                score -= abs(ty - sym_target_y) * params["symmetry_weight"]
            
            dist_x_to_ball = abs(tx - ball.x); score -= abs(dist_x_to_ball - 2.5) * params["ball_dist_weight"]
            score += (-tx) * params["forward_weight"]
            
            pass_path = (ball.x, ball.y, tx, ty)
            shot_path = (base_x, ty, goal_x, 0.0)
            move_path = (robot.x, robot.y, tx, ty) # Robot to Target
            
            for opp in opponents:
                if opp.label != "Opponent": continue
                cf = confidence_factor(opp.last_seen_sec_ago, params["opp_memory_sec"])
                if cf <= 0.0: continue
                
                # 9. Pass Path Penalty (Condition: Ball seen recently < 3000ms, simplify to always for now or add param)
                dist_pass = point_to_segment_distance(opp.pos.x, opp.pos.y, *pass_path)
                if dist_pass < params["path_margin"]: 
                    score -= (params["path_margin"] - dist_pass) * params["pass_penalty_weight"] * cf
                
                # 10. Shot Path Penalty
                dist_shot = point_to_segment_distance(opp.pos.x, opp.pos.y, *shot_path)
                if dist_shot < params["path_margin"]: 
                    score -= (params["path_margin"] - dist_shot) * params["shot_penalty_weight"] * cf
                    
                # 11. Movement Path Penalty
                dist_robot_target = np.hypot(tx - robot.x, ty - robot.y)
                if dist_robot_target > 0.1:
                    dist_move = point_to_segment_distance(opp.pos.x, opp.pos.y, *move_path)
                    if dist_move < params["path_margin"]:
                         score -= (params["path_margin"] - dist_move) * params["movement_penalty_weight"] * cf

            # 12. Goal Post Avoidance (New)
            goal_w_half = params["goal_width"] / 2.0
            dist_l_post = np.hypot(tx - goal_x, ty - (-goal_w_half))
            dist_r_post = np.hypot(tx - goal_x, ty - (goal_w_half))
            
            if dist_l_post < 0.5: score -= (0.5 - dist_l_post) * 20.0
            if dist_r_post < 0.5: score -= (0.5 - dist_r_post) * 20.0
            
            if score > best_score:
                best_score = score
                best_pos = (tx, ty)
    return best_pos, best_score
