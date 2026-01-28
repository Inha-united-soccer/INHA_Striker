
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
import numpy as np
import time

# Import Logic and Params
import sim_params as CFG
import sim_logic as Logic

# --- Configuration ---
PASS_PARAMS = CFG.PASS_PARAMS.copy()
ST_PARAMS = CFG.ST_PARAMS.copy()
SIM = CFG.SIM.copy()
DT = SIM["dt"]
PLAYER_SPEED = SIM["player_speed"]
USER_STEP = SIM["user_step"] * 2.0 

# Game Constants
PASS_CONFIRM_TIME = 0.5 

# --- Global State ---
class GameState:
    def __init__(self):
        self.ball = Logic.Pose2D(0.0, 0.0)
        self.passer = Logic.Pose2D(0.1, 0.0)
        self.striker = Logic.Pose2D(-3.0, 3.0)
        self.opp_user = Logic.Pose2D(-1.8, 0.5)
        self.opp_gk = Logic.Pose2D(-3.8, 0.5)
        
        # Scoring
        self.score_goals = 0
        self.score_fails = 0
        
        # Logic State
        self.pfound = False
        self.pass_target = Logic.Pose2D(0, 0)
        self.best_pos = (0, 0)
        self.best_score = 0.0
        
        self.pass_found_timer = 0.0
        self.pass_target_active = Logic.Pose2D(0, 0)
        
        # "PLAYING", "BALL_MOVING", "RESETTING"
        self.game_state = "PLAYING"
        self.reset_timer = 0
        self.msg = ""
        
        # Keyboard input state
        self.keys = {'w': False, 'a': False, 's': False, 'd': False}

state = GameState()

# --- Simulation Logic ---
def update_logic():
    # 1. Update Opponent (User Control) - Always allow movement
    if state.keys['w']: state.opp_user.y += USER_STEP
    if state.keys['s']: state.opp_user.y -= USER_STEP
    if state.keys['a']: state.opp_user.x -= USER_STEP
    if state.keys['d']: state.opp_user.x += USER_STEP
    
    # Clamp
    state.opp_user.x = Logic.clamp(state.opp_user.x, -5.0, 5.0)
    state.opp_user.y = Logic.clamp(state.opp_user.y, -3.5, 3.5)
    
    opponents = [
        Logic.Opponent(state.opp_gk, 0.0, label="GK"),
        Logic.Opponent(state.opp_user, 0.0, label="Opponent") 
    ]
    
    if state.game_state == "PLAYING":
        # Ball held by passer
        state.ball.x, state.ball.y = state.passer.x, state.passer.y
        
        # AI Move (Striker)
        best_pos_tuple, best_score = Logic.compute_striker_costmap(
            state.striker, state.ball, opponents, ST_PARAMS
        )
        state.best_pos = best_pos_tuple
        state.best_score = best_score
        
        st_target = Logic.Pose2D(best_pos_tuple[0], best_pos_tuple[1])
        state.striker = Logic.move_towards(state.striker, st_target, PLAYER_SPEED, DT)
        
        # Pass Check
        teammates = [Logic.Teammate(1, state.passer), Logic.Teammate(2, state.striker)]
        best_tm = Logic.select_best_teammate(state.ball, teammates, 1, PASS_PARAMS)
        
        state.pfound = False
        if best_tm:
            ptx, pty, psc = Logic.compute_pass_costmap(state.ball, best_tm.pos, opponents, PASS_PARAMS)
            if psc >= PASS_PARAMS["score_threshold"]:
                state.pass_found_timer += DT
                if state.pass_found_timer > PASS_CONFIRM_TIME:
                    # PASS!
                    state.game_state = "BALL_MOVING"
                    state.pass_target_active = Logic.Pose2D(ptx, pty)
                    state.pass_found_timer = 0
                state.pfound = True
                state.pass_target = Logic.Pose2D(ptx, pty)
            else:
                state.pass_found_timer = 0
                
    elif state.game_state == "BALL_MOVING":
        # Ball Travels
        state.ball = Logic.move_towards(state.ball, state.pass_target_active, 8.0, DT) # Fast ball
        
        # Check Intercept (Opponents)
        caught = False
        for opp in opponents:
            d = np.hypot(state.ball.x - opp.pos.x, state.ball.y - opp.pos.y)
            if d < 0.4:
                caught = True
                break
        
        if caught:
            state.game_state = "RESETTING"
            state.score_fails += 1
            state.msg = "INTERCEPTED!"
            state.reset_timer = 1.0 # Pause 1s
            
        # Check Receipt (Striker)
        d_tm = np.hypot(state.ball.x - state.striker.x, state.ball.y - state.striker.y)
        if d_tm < 0.4:
            state.game_state = "RESETTING"
            state.score_goals += 1
            state.msg = "GOAL!"
            state.reset_timer = 1.0
            
        # Check Miss (Stop at target)
        if abs(state.ball.x - state.pass_target_active.x) < 1e-5 and abs(state.ball.y - state.pass_target_active.y) < 1e-5:
            if state.game_state == "BALL_MOVING":
                state.game_state = "RESETTING"
                state.msg = "MISSED!"
                state.reset_timer = 1.0
                
    elif state.game_state == "RESETTING":
        state.reset_timer -= DT
        if state.reset_timer <= 0:
            # Reset Positions
            state.ball = Logic.Pose2D(0.0, 0.0)
            state.striker = Logic.Pose2D(-3.0, 3.0)
            state.opp_user = Logic.Pose2D(-1.8, 0.5)
            state.game_state = "PLAYING"
            state.msg = ""

# --- Visualization ---
fig, ax = plt.subplots(figsize=(10, 7))
ax.set_xlim(-5.5, 5.5)
ax.set_ylim(-4.0, 4.0)
ax.set_aspect('equal')
ax.set_facecolor('#228B22') # Green field

# Field markings
ax.plot([0, 0], [-3.5, 3.5], 'white', linewidth=2)
ax.plot([-5, 5, 5, -5, -5], [-3.5, -3.5, 3.5, 3.5, -3.5], 'white', linewidth=2)

# Entities
striker_patch = patches.Circle((state.striker.x, state.striker.y), 0.3, color='#00FFFF', label='Striker')
passer_patch = patches.Rectangle((state.passer.x - 0.2, state.passer.y - 0.2), 0.4, 0.4, color='blue', label='Passer')
ball_patch = patches.Circle((state.ball.x, state.ball.y), 0.15, color='orange', label='Ball')
opp_user_patch = patches.RegularPolygon((state.opp_user.x, state.opp_user.y), numVertices=4, radius=0.3, color='red', label='Defender (You)')
opp_gk_patch = patches.RegularPolygon((state.opp_gk.x, state.opp_gk.y), numVertices=4, radius=0.2, orientation=np.pi/4, color='darkred', label='GK')

# Indicators
target_patch = patches.Circle((0, 0), 0.1, color='white', alpha=0.5, label='Target (AI)')
pass_line, = ax.plot([], [], 'yellow', linewidth=3, visible=False) # Potential Pass Line

ax.add_patch(striker_patch)
ax.add_patch(passer_patch)
ax.add_patch(ball_patch)
ax.add_patch(opp_user_patch)
ax.add_patch(opp_gk_patch)
ax.add_patch(target_patch)

# Text
info_text = ax.text(-5.0, 3.7, '', color='white', fontsize=12, fontweight='bold')
msg_text = ax.text(0, 0, '', color='yellow', fontsize=24, fontweight='bold', ha='center', va='center')

# Legend
ax.legend(loc='lower right', facecolor='white', framealpha=0.5)

def init():
    return striker_patch, passer_patch, ball_patch, opp_user_patch, opp_gk_patch, target_patch, pass_line, info_text, msg_text

def animate(frame):
    update_logic()
    
    # Update positions
    striker_patch.center = (state.striker.x, state.striker.y)
    opp_user_patch.xy = (state.opp_user.x, state.opp_user.y)
    ball_patch.center = (state.ball.x, state.ball.y)
    target_patch.center = state.best_pos
    
    # Pass Indicator
    if state.game_state == "PLAYING" and state.pfound:
        pass_line.set_data([state.ball.x, state.pass_target.x], [state.ball.y, state.pass_target.y])
        pass_line.set_visible(True)
    else:
        pass_line.set_visible(False)
        
    # Text
    info_text.set_text(f"GOALS: {state.score_goals} | INTERCEPTED: {state.score_fails}")
    msg_text.set_text(state.msg)
    
    return striker_patch, passer_patch, ball_patch, opp_user_patch, opp_gk_patch, target_patch, pass_line, info_text, msg_text

# Keyboard Input
def on_key_press(event):
    if event.key in state.keys:
        state.keys[event.key] = True

def on_key_release(event):
    if event.key in state.keys:
        state.keys[event.key] = False

fig.canvas.mpl_connect('key_press_event', on_key_press)
fig.canvas.mpl_connect('key_release_event', on_key_release)

ani = animation.FuncAnimation(fig, animate, init_func=init, interval=50, blit=True)

plt.title("Soccer Defender Game (Python Matplotlib)")
plt.show()
