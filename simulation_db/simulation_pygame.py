#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Pygame-based Soccer Simulator v3.0 (Gamified)
=============================================
Goal: As the DEFENDER (Red), try to INTERCEPT the ball.
The AI STRIKER (Blue) tries to get open. If open, the PASSER passes.

[Controls]
- ARROWS: Move User (Defender)
- TAB: Tuning Mode
- R: Record Run for Leaderboard
"""

import pygame
import sys
import math
import csv
import time
import json
import random
from dataclasses import dataclass, asdict
from typing import List, Optional, Tuple, Dict
import numpy as np

import sim_params as CFG
from db_manager import DBManager

# --- Config ---
PASS_PARAMS = CFG.PASS_PARAMS.copy()
ST_PARAMS = CFG.ST_PARAMS.copy()
SIM = CFG.SIM.copy()

WIN_W = 1200
WIN_H = 800
SCALE = 100.0

COLOR_BG = (34, 139, 34) # Forest Green
COLOR_UI_BG = (40, 44, 52)
COLOR_TEXT = (255, 255, 255)
COLOR_HIGHLIGHT = (255, 215, 0) # Gold
COLOR_LINE = (240, 240, 240)
COLOR_BALL = (255, 140, 0) # Dark Orange
COLOR_TEAM = (30, 144, 255) # Dodger Blue
COLOR_OPP = (220, 20, 60) # Crimson
COLOR_TARGET = (0, 255, 255)

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

# --- AI Logic (Same as before) ---
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
            
            dist_to_defender = 0.0; defenders_in_box = 0
            for opp in opponents:
                 if abs(opp.pos.x - goal_x) < 4.0:
                     d = np.hypot(ty - opp.pos.y, tx - opp.pos.x)
                     d = min(d, params["defender_dist_cap"]); dist_to_defender += d; defenders_in_box += 1
            if defenders_in_box > 0: dist_to_defender /= defenders_in_box
            score += dist_to_defender * params["defender_dist_weight"]
            
            dist_x_to_ball = abs(tx - ball.x); score -= abs(dist_x_to_ball - 2.5) * params["ball_dist_weight"]
            score += (-tx) * params["forward_weight"]
            
            pass_path = (ball.x, ball.y, tx, ty)
            for opp in opponents:
                cf = confidence_factor(opp.last_seen_sec_ago, params["opp_memory_sec"])
                if cf <= 0.0: continue
                dist_pass = point_to_segment_distance(opp.pos.x, opp.pos.y, *pass_path)
                if dist_pass < params["path_margin"]: score -= (params["path_margin"] - dist_pass) * params["penalty_weight"] * cf
            
            dist_robot_target = np.hypot(tx - robot.x, ty - robot.y)
            if dist_robot_target > 0.1:
                # Movement penalty simplified
                pass 
            
            if score > best_score:
                best_score = score
                best_pos = (tx, ty)
    return best_pos, best_score

# --- UI ---
def to_screen(field_x, field_y):
    sx = WIN_W / 2 + field_x * SCALE
    sy = WIN_H / 2 - field_y * SCALE 
    return int(sx), int(sy)

def draw_field(screen):
    screen.fill(COLOR_BG)
    hlx = PASS_PARAMS["field_half_length"]; hly = PASS_PARAMS["field_half_width"]
    p1 = to_screen(-hlx, hly); p2 = to_screen(hlx, hly); p3 = to_screen(hlx, -hly); p4 = to_screen(-hlx, -hly)
    pygame.draw.lines(screen, COLOR_LINE, True, [p1, p2, p3, p4], 2)
    pygame.draw.line(screen, COLOR_LINE, to_screen(0, hly), to_screen(0, -hly), 1)

def draw_entity(screen, pos, color, radius_m, font, text=None):
    sp = to_screen(pos.x, pos.y)
    pygame.draw.circle(screen, color, sp, int(radius_m * SCALE))
    if text:
        img = font.render(text, True, COLOR_TEXT)
        screen.blit(img, (sp[0]+12, sp[1]-12))

# --- Main ---
def main():
    pygame.init()
    screen = pygame.display.set_mode((WIN_W, WIN_H))
    pygame.display.set_caption("Soccer Logic Simulator v3.0 (Game Mode)")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("Arial", 16)
    font_lg = pygame.font.SysFont("Arial", 32)
    font_xl = pygame.font.SysFont("Arial", 64)
    
    db = DBManager()
    
    mode = "SIMULATION"
    game_state = "PLAYING" # PLAYING, BALL_MOVING, RESETTING
    
    ball = Pose2D(0.0, 0.0)
    passer = Pose2D(0.1, 0.0)
    striker = Pose2D(-3.0, 3.0)
    gk = Pose2D(3.5, 0.0)
    
    opp_user = Pose2D(-1.8, 0.5)
    opponents = [Opponent(Pose2D(-3.8, 0.5), 0.0), Opponent(opp_user, 0.0)]
    
    # Game Stats
    score_goals = 0
    score_fails = 0
    pass_target_active = Pose2D(0,0)
    
    recording = False
    rec_start_time = 0
    rec_data = []
    
    # Tuning
    tuning_keys = []
    for k in ST_PARAMS: tuning_keys.append(("ST", k))
    for k in PASS_PARAMS: tuning_keys.append(("PASS", k))
    tuning_idx = 0
    
    # Pass delay logic
    pass_found_timer = 0.0
    PASS_CONFIRM_TIME = 0.5 # Needs to stay open for 0.5s
    
    running = True
    while running:
        dt = SIM["dt"]
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_TAB: mode = "TUNING" if mode == "SIMULATION" else "SIMULATION"
                if mode == "SIMULATION":
                    if event.key == pygame.K_r:
                        if not recording:
                            recording = True; rec_start_time = time.time(); rec_data = []; score_goals = 0; score_fails = 0
                            print("Recording Started")
                        else:
                            recording = False; duration = time.time() - rec_start_time
                            db.save_run(duration, rec_data, PASS_PARAMS, ST_PARAMS)
                            print("Recording Saved to DB")
                elif mode == "TUNING":
                    if event.key == pygame.K_UP: tuning_idx = (tuning_idx - 1) % len(tuning_keys)
                    elif event.key == pygame.K_DOWN: tuning_idx = (tuning_idx + 1) % len(tuning_keys)
                    elif event.key == pygame.K_LEFT or event.key == pygame.K_RIGHT:
                        cat, key = tuning_keys[tuning_idx]
                        target_dict = ST_PARAMS if cat == "ST" else PASS_PARAMS
                        delta = 0.5 if key.endswith("weight") else 0.1
                        if event.key == pygame.K_LEFT: delta *= -1
                        target_dict[key] += delta

        # Game Logic
        keys = pygame.key.get_pressed()
        step = SIM["user_step"]
        if keys[pygame.K_UP]: opp_user.y += step
        if keys[pygame.K_DOWN]: opp_user.y -= step
        if keys[pygame.K_LEFT]: opp_user.x -= step
        if keys[pygame.K_RIGHT]: opp_user.x += step
        
        # Link Opponent Objects
        opponents[1].pos = opp_user
        
        # --- State Machine ---
        if game_state == "PLAYING":
            ball.x, ball.y = passer.x, passer.y # Ball holds by passer
            
            # AI Move
            best_ofb_pos, best_ofb_sc = compute_striker_costmap(striker, ball, opponents, ST_PARAMS)
            st_target = Pose2D(best_ofb_pos[0], best_ofb_pos[1])
            striker = move_towards(striker, st_target, SIM["player_speed"], dt)
            
            # Pass Check
            teammates = [Teammate(1, Pose2D(passer.x, passer.y)), Teammate(2, Pose2D(striker.x, striker.y))]
            best_tm = select_best_teammate(ball, teammates, 1, PASS_PARAMS)
            pfound = False
            if best_tm:
                 ptx, pty, psc = compute_pass_costmap(ball, best_tm.pos, opponents, PASS_PARAMS)
                 if psc >= PASS_PARAMS["score_threshold"]:
                     pass_found_timer += dt
                     if pass_found_timer > PASS_CONFIRM_TIME:
                         # PASS!
                         game_state = "BALL_MOVING"
                         pass_target_active = Pose2D(ptx, pty)
                         pass_found_timer = 0
                     pfound = True
                 else:
                     pass_found_timer = 0
            
        elif game_state == "BALL_MOVING":
            # Ball travels
            ball = move_towards(ball, pass_target_active, 8.0, dt) # Fast ball
            
            # Check Intercept
            for opp in opponents:
                d = np.hypot(ball.x - opp.pos.x, ball.y - opp.pos.y)
                if d < 0.4: # Caught
                    game_state = "RESETTING"
                    score_fails += 1
                    msg = "INTERCEPTED!"
                    
            # Check Receipt
            d_tm = np.hypot(ball.x - striker.x, ball.y - striker.y)
            if d_tm < 0.4:
                game_state = "RESETTING"
                score_goals += 1
                msg = "GOAL!"
                
            # Check Miss (Stop at target)
            if ball.x == pass_target_active.x and ball.y == pass_target_active.y:
                # If no one caught it, it's a miss
                if game_state == "BALL_MOVING":
                    game_state = "RESETTING"
                    # No score change, just reset logic? or fail? Let's say fail for now
                    msg = "MISSED!"
                    
        elif game_state == "RESETTING":
            # Show message overlay?
            # Simple reset
            ball = Pose2D(0,0)
            striker.x, striker.y = -3.0, 3.0
            opp_user.x, opp_user.y = -1.8, 0.5
            game_state = "PLAYING"
            
        # Logging
        if recording and game_state == "PLAYING":
             rec_data.append({
                 "t": time.time() - rec_start_time,
                 "ball": (float(ball.x), float(ball.y)),
                 "ofb_score": float(best_ofb_sc),
                 "goals": int(score_goals)
             })

        # --- Draw ---
        screen.fill(COLOR_BG)
        
        if mode == "SIMULATION":
            draw_field(screen)
            draw_entity(screen, passer, COLOR_TEAM, 0.2, font, "Passer")
            draw_entity(screen, striker, COLOR_TEAM, 0.2, font, "Striker")
            draw_entity(screen, ball, COLOR_BALL, 0.15, font)
            draw_entity(screen, opp_user, COLOR_OPP, 0.2, font, "YOU")
            draw_entity(screen, opponents[0].pos, COLOR_OPP, 0.2, font, "GK")
            
            if game_state == "PLAYING":
                draw_entity(screen, st_target, COLOR_TARGET, 0.1, font)
                if pfound:
                    pygame.draw.line(screen, COLOR_HIGHLIGHT, to_screen(ball.x,ball.y), to_screen(ptx,pty), 3)

            # Scoreboard
            sb = font_lg.render(f"GOALS: {score_goals}   |   FAILS: {score_fails}", True, COLOR_HIGHLIGHT)
            screen.blit(sb, (WIN_W//2 - sb.get_width()//2, 20))
            
            # Rec Status
            col = (255,0,0) if recording else (200,200,200)
            screen.blit(font.render(f"REC: {'ON' if recording else 'OFF'} (R)", True, col), (10, 10))
            
        elif mode == "TUNING":
            screen.fill(COLOR_UI_BG)
            screen.blit(font_lg.render("TUNING MODE", True, COLOR_HIGHLIGHT), (40, 20))
            y_off = 80
            for i, (cat, key) in enumerate(tuning_keys):
                val = ST_PARAMS[key] if cat == "ST" else PASS_PARAMS[key]
                col = COLOR_HIGHLIGHT if i == tuning_idx else COLOR_TEXT
                screen.blit(font.render(f"{cat} {key}: {val:.2f}", True, col), (60, y_off))
                y_off += 25
                if y_off > WIN_H: break

        pygame.display.flip()
        clock.tick(30)

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
