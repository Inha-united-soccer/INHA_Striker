
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
import numpy as np
import time

# Import Logic and Params
import sim_params as CFG
import sim_logic as Logic

# --- Configuration ---
# You can tweak these here or in sim_params.py
DT = CFG.SIM["dt"]
PLAYER_SPEED = CFG.SIM["player_speed"]
USER_STEP = CFG.SIM["user_step"] * 2.0 # Slightly faster for keyboard

# --- Global State ---
class GameState:
    def __init__(self):
        self.ball = Logic.Pose2D(0.0, 0.0)
        self.passer = Logic.Pose2D(0.1, 0.0)
        self.striker = Logic.Pose2D(-3.0, 3.0)
        self.opp_user = Logic.Pose2D(-1.8, 0.5)
        self.opp_gk = Logic.Pose2D(-3.8, 0.5)
        
        self.score = 0.0
        self.pfound = False
        self.pass_target = Logic.Pose2D(0, 0)
        self.best_pos = (0, 0)
        
        self.st_params = CFG.ST_PARAMS.copy()
        self.pass_params = CFG.PASS_PARAMS.copy()
        
        # Keyboard input state
        self.keys = {'w': False, 'a': False, 's': False, 'd': False}

state = GameState()

# --- Simulation Logic ---
def update_logic():
    # 1. Update Opponent (User Control)
    if state.keys['w']: state.opp_user.y += USER_STEP
    if state.keys['s']: state.opp_user.y -= USER_STEP
    if state.keys['a']: state.opp_user.x -= USER_STEP
    if state.keys['d']: state.opp_user.x += USER_STEP
    
    # Clamp
    state.opp_user.x = Logic.clamp(state.opp_user.x, -5.0, 5.0)
    state.opp_user.y = Logic.clamp(state.opp_user.y, -3.5, 3.5)
    
    # 2. AI Logic (Striker Off-the-ball)
    opponents = [
        Logic.Opponent(state.opp_gk, 0.0, label="GK"),
        Logic.Opponent(state.opp_user, 0.0, label="Opponent") # Only label="Opponent" affects pass logic in sim_logic
    ]
    
    # Striker Costmap
    best_pos_tuple, best_score = Logic.compute_striker_costmap(
        state.striker, state.ball, opponents, state.st_params
    )
    state.best_pos = best_pos_tuple
    state.score = best_score
    
    target = Logic.Pose2D(best_pos_tuple[0], best_pos_tuple[1])
    
    # Move Striker
    state.striker = Logic.move_towards(state.striker, target, PLAYER_SPEED, DT)
    
    # 3. Pass Logic
    teammates = [
        Logic.Teammate(1, state.passer, label="Passer"),
        Logic.Teammate(2, state.striker, label="Striker")
    ]
    
    best_tm = Logic.select_best_teammate(state.ball, teammates, 1, state.pass_params)
    
    state.pfound = False
    if best_tm:
        ptx, pty, psc = Logic.compute_pass_costmap(state.ball, best_tm, opponents, state.pass_params)
        if psc >= state.pass_params["score_threshold"]:
            state.pfound = True
            state.pass_target = Logic.Pose2D(ptx, pty)

# --- Visualization ---
fig, ax = plt.subplots(figsize=(10, 7))
ax.set_xlim(-5.5, 5.5)
ax.set_ylim(-4.0, 4.0)
ax.set_aspect('equal')
ax.set_facecolor('#228B22') # Green field

# Field markings
# Center line
ax.plot([0, 0], [-3.5, 3.5], 'white', linewidth=2)
# Borders
ax.plot([-5, 5, 5, -5, -5], [-3.5, -3.5, 3.5, 3.5, -3.5], 'white', linewidth=2)

# Entities
striker_patch = patches.Circle((state.striker.x, state.striker.y), 0.3, color='#00FFFF', label='Striker')
passer_patch = patches.Rectangle((state.passer.x - 0.2, state.passer.y - 0.2), 0.4, 0.4, color='blue', label='Passer')
ball_patch = patches.Circle((state.ball.x, state.ball.y), 0.15, color='orange', label='Ball')
opp_user_patch = patches.RegularPolygon((state.opp_user.x, state.opp_user.y), numVertices=4, radius=0.3, color='red', label='Defender (You)')
opp_gk_patch = patches.RegularPolygon((state.opp_gk.x, state.opp_gk.y), numVertices=4, radius=0.2, orientation=np.pi/4, color='darkred', label='GK')

target_patch = patches.Circle((0, 0), 0.1, color='white', alpha=0.5, label='Target')
pass_target_patch = patches.RegularPolygon((0, 0), numVertices=3, radius=0.2, color='yellow', label='Pass Target', visible=False)

ax.add_patch(striker_patch)
ax.add_patch(passer_patch)
ax.add_patch(ball_patch)
ax.add_patch(opp_user_patch)
ax.add_patch(opp_gk_patch)
ax.add_patch(target_patch)
ax.add_patch(pass_target_patch)

# Info Text
info_text = ax.text(-5.0, 3.7, '', color='white', fontsize=10)

# Legend
ax.legend(loc='upper right', facecolor='white', framealpha=0.5)

def init():
    return striker_patch, passer_patch, ball_patch, opp_user_patch, opp_gk_patch, target_patch, pass_target_patch, info_text

def animate(frame):
    update_logic()
    
    # Update positions
    striker_patch.center = (state.striker.x, state.striker.y)
    
    # Update user patch (RegularPolygon center is xy)
    opp_user_patch.xy = (state.opp_user.x, state.opp_user.y)
    
    # Update AI Target
    target_patch.center = state.best_pos
    
    # Update Pass Target
    if state.pfound:
        pass_target_patch.xy = (state.pass_target.x, state.pass_target.y)
        pass_target_patch.set_visible(True)
    else:
        pass_target_patch.set_visible(False)
        
    info_text.set_text(f"Score: {state.score:.2f} | WASD to move opponent")
    
    return striker_patch, passer_patch, ball_patch, opp_user_patch, opp_gk_patch, target_patch, pass_target_patch, info_text

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

plt.title("Soccer Simulation (Python Standalone)")
plt.show()
