
import streamlit as st
import time
import pandas as pd
import altair as alt
import numpy as np

# Import our Logic and Params
import sim_params as CFG
import sim_logic as Logic
from db_manager import DBManager

st.set_page_config(page_title="Soccer Sim Web", layout="wide")

# Initialize Session State
if "running" not in st.session_state: st.session_state["running"] = False
if "recording" not in st.session_state: st.session_state["recording"] = False
if "rec_data" not in st.session_state: st.session_state["rec_data"] = []
if "rec_start_time" not in st.session_state: st.session_state["rec_start_time"] = 0
if "ball" not in st.session_state: st.session_state["ball"] = Logic.Pose2D(0.0, 0.0)
if "passer" not in st.session_state: st.session_state["passer"] = Logic.Pose2D(0.1, 0.0)
if "striker" not in st.session_state: st.session_state["striker"] = Logic.Pose2D(-3.0, 3.0)
if "opp_user" not in st.session_state: st.session_state["opp_user"] = Logic.Pose2D(-1.8, 0.5)
if "game_stats" not in st.session_state: st.session_state["game_stats"] = {"goals": 0, "fails": 0}

db = DBManager()

st.title("⚽️ Soccer Simulation Dashboard")

# Initialize Slider State
if "def_x" not in st.session_state: st.session_state["def_x"] = float(st.session_state["opp_user"].x)
if "def_y" not in st.session_state: st.session_state["def_y"] = float(st.session_state["opp_user"].y)

# Sidebar - Parameters
st.sidebar.title("🛠 Logic Settings")

def get_params_from_sidebar():
    new_st_params = CFG.ST_PARAMS.copy()
    new_pass_params = CFG.PASS_PARAMS.copy()
    
    with st.sidebar.expander("Striker Logic", expanded=True):
        new_st_params["base_x_weight"] = st.slider("Base X (Pos)", 0.0, 20.0, float(CFG.ST_PARAMS["base_x_weight"]))
        new_st_params["center_y_weight"] = st.slider("Center Y", 0.0, 20.0, float(CFG.ST_PARAMS["center_y_weight"]))
        new_st_params["defender_dist_weight"] = st.slider("Avoid Defender", 0.0, 50.0, float(CFG.ST_PARAMS["defender_dist_weight"]))
        new_st_params["symmetry_weight"] = st.slider("Symmetry (Space)", 0.0, 30.0, float(CFG.ST_PARAMS["symmetry_weight"]))
        new_st_params["forward_weight"] = st.slider("Forward Bias", 0.0, 20.0, float(CFG.ST_PARAMS["forward_weight"]))
        new_st_params["ball_dist_weight"] = st.slider("Ball Dist", 0.0, 20.0, float(CFG.ST_PARAMS["ball_dist_weight"]))
        
        st.caption("Penalties")
        new_st_params["pass_penalty_weight"] = st.slider("Pass Block Pen", 0.0, 50.0, float(CFG.ST_PARAMS["pass_penalty_weight"]))
        new_st_params["shot_penalty_weight"] = st.slider("Shot Block Pen", 0.0, 50.0, float(CFG.ST_PARAMS["shot_penalty_weight"]))
        new_st_params["movement_penalty_weight"] = st.slider("Move Block Pen", 0.0, 50.0, float(CFG.ST_PARAMS["movement_penalty_weight"]))
        
        st.caption("Stability (Hysteresis)")
        new_st_params["hysteresis_x_weight"] = st.slider("Hysteresis X", 0.0, 10.0, float(CFG.ST_PARAMS["hysteresis_x_weight"]))
        new_st_params["hysteresis_y_weight"] = st.slider("Hysteresis Y", 0.0, 10.0, float(CFG.ST_PARAMS["hysteresis_y_weight"]))
    
    with st.sidebar.expander("Pass Logic (Teammates)", expanded=False):
        new_pass_params["score_threshold"] = st.slider("Pass Threshold", 0.0, 10.0, float(CFG.PASS_PARAMS["score_threshold"]))
        new_pass_params["opp_penalty"] = st.slider("Opponent Penalty", 0.0, 50.0, float(CFG.PASS_PARAMS["opp_penalty"]))

    return new_st_params, new_pass_params

st_params, pass_params = get_params_from_sidebar()

# Layout
col_ctrl, col_stat = st.columns([1, 2])
with col_ctrl:
    st.markdown("### 🎮 Controls")
    
    # 1. Sim Control
    c1, c2 = st.columns(2)
    if not st.session_state["running"]:
        if c1.button("▶️ START", use_container_width=True):
            st.session_state["running"] = True
            st.rerun()
    else:
        if c1.button("⏹ STOP", use_container_width=True):
            st.session_state["running"] = False
            st.session_state["recording"] = False
            st.rerun()

    # 2. Recording
    if st.session_state["running"]:
        if not st.session_state["recording"]:
            if c2.button("⏺ RECORD", use_container_width=True):
                st.session_state["recording"] = True
                st.session_state["rec_data"] = []
                st.session_state["rec_start_time"] = time.time()
                st.rerun()
        else:
            if c2.button("💾 SAVE", use_container_width=True, type="primary"):
                st.session_state["recording"] = False
                duration = time.time() - st.session_state["rec_start_time"]
                run_id = db.save_run(duration, st.session_state["rec_data"], pass_params, st_params)
                st.success(f"Run {run_id} Saved!")
                st.rerun()

    # 3. Sliders (The "UI to move")
    st.markdown("---")
    st.markdown("### ♟️ Move Defender")
    
    def_x = st.slider("↔️ X Position", -5.0, 5.0, key="def_x")
    def_y = st.slider("↕️ Y Position", -3.5, 3.5, key="def_y")
    
    # Update State
    st.session_state["opp_user"].x = def_x
    st.session_state["opp_user"].y = def_y

with col_stat:
    c1, c2, c3 = st.columns(3)
    c1.metric("Goals", st.session_state["game_stats"]["goals"])
    c2.metric("Interceptions", st.session_state["game_stats"]["fails"])
    if st.session_state["recording"]:
        c3.warning("🔴 RECORDING... (Press SAVE to Upload)")

# --- Simulation Logic ---
def run_simulation_step():
    ball = st.session_state["ball"]
    striker = st.session_state["striker"]
    opp_user = st.session_state["opp_user"]
    passer = st.session_state["passer"]
    
    opponents = [Logic.Opponent(Logic.Pose2D(-3.8, 0.5), 0.0), Logic.Opponent(opp_user, 0.0)]
    
    # AI Logic
    best_pos, best_score = Logic.compute_striker_costmap(striker, ball, opponents, st_params)
    target = Logic.Pose2D(best_pos[0], best_pos[1])
    
    dt = 0.1
    speed = 0.5
    striker = Logic.move_towards(striker, target, speed, dt)
    st.session_state["striker"] = striker
    
    # Pass Logic
    teammates = [Logic.Teammate(1, passer), Logic.Teammate(2, striker)]
    pfound = False
    pass_target = Logic.Pose2D(0,0)
    
    best_tm = Logic.select_best_teammate(ball, teammates, 1, pass_params)
    if best_tm:
        ptx, pty, psc = Logic.compute_pass_costmap(ball, best_tm.pos, opponents, pass_params)
        if psc >= pass_params["score_threshold"]:
            pfound = True
            pass_target = Logic.Pose2D(ptx, pty)
            
            # Simple Scoring Interaction
            d_def = np.hypot(striker.x - opp_user.x, striker.y - opp_user.y)
            # In real logic we wait, here we just show feedback
            
    return best_pos, best_score, pfound, pass_target

# Run 1 Step
best_pos, best_score, pfound, pass_target = run_simulation_step()

# Logging
if st.session_state["recording"]:
    st.session_state["rec_data"].append({
        "t": time.time(),
        "ofb_score": float(best_score),
        "striker": (float(st.session_state["striker"].x), float(st.session_state["striker"].y))
    })

# --- Altair Visuals (Rich Field) ---

# 1. Background (Green Field)
domain_x = [-5.0, 5.0]
domain_y = [-3.5, 3.5]
bg_df = pd.DataFrame({'x': [0], 'y': [0]}) # Dummy
field_bg = alt.Chart(bg_df).mark_rect(color='#228B22', opacity=0.8).encode(
    x=alt.value(0), x2=alt.value(700), # Pixel width matching properties below
    y=alt.value(0), y2=alt.value(500)
)
# Note: mark_rect with pixel values is tricky. 
# Better: Just set the chart background in config or use a huge point.
# Let's use standard domain encoding.
field_bg = alt.Chart(pd.DataFrame({'x': [-5, 5], 'y': [-3.5, 3.5]})).mark_rect(color='#228B22', opacity=0.3).encode(
    x='x', y='y'
)

# 2. Field Lines
lines_data = [
    # Borders
    {'x1': -5, 'y1': -3.5, 'x2': 5, 'y2': -3.5},
    {'x1': -5, 'y1': 3.5, 'x2': 5, 'y2': 3.5},
    {'x1': -5, 'y1': -3.5, 'x2': -5, 'y2': 3.5},
    {'x1': 5, 'y1': -3.5, 'x2': 5, 'y2': 3.5},
    # Center Line
    {'x1': 0, 'y1': -3.5, 'x2': 0, 'y2': 3.5},
]
lines_df = pd.DataFrame(lines_data)
field_lines = alt.Chart(lines_df).mark_rule(color='white', strokeWidth=2).encode(
    x='x1', x2='x2', y='y1', y2='y2'
)

# 3. Entities
s = st.session_state["striker"]
p = st.session_state["passer"]
b = st.session_state["ball"]
u = st.session_state["opp_user"]

entities = [
    {"x": s.x, "y": s.y, "type": "Striker (AI)", "color": "#00FFFF", "size": 300, "shape": "circle"},
    {"x": p.x, "y": p.y, "type": "Passer", "color": "#0000FF", "size": 200, "shape": "square"},
    {"x": b.x, "y": b.y, "type": "Ball", "color": "#FFA500", "size": 150, "shape": "circle"},
    {"x": u.x, "y": u.y, "type": "Defender (You)", "color": "#FF0000", "size": 400, "shape": "diamond"},
    {"x": -3.8, "y": 0.5, "type": "GK", "color": "#8B0000", "size": 200, "shape": "cross"},
    {"x": best_pos[0], "y": best_pos[1], "type": "Target", "color": "#FFFFFF", "size": 100, "shape": "cross"},
]
if pfound:
     entities.append({"x": pass_target.x, "y": pass_target.y, "type": "Pass Target", "color": "#FFFF00", "size": 100, "shape": "triangle"})

ent_df = pd.DataFrame(entities)
players = alt.Chart(ent_df).mark_point(filled=True).encode(
    x=alt.X('x', scale=alt.Scale(domain=[-5.5, 5.5])),
    y=alt.Y('y', scale=alt.Scale(domain=[-4, 4])),
    color=alt.Color('color', scale=None),
    size=alt.Size('size', scale=None),
    shape=alt.Shape('shape', scale=None),
    tooltip=['type']
)

# Combine Layers
# Order: Lines -> Players (Background handled by theme or style usually, but here lines give structure)
chart = (field_lines + players).properties(
    width=700, height=500,
    title=f"Field View (Score: {best_score:.2f}) {'[OPEN!]' if pfound else ''}"
).configure_view(
    strokeWidth=0,
    fill='#228B22' # Set Chart Background to Green
).configure_axis(
    grid=False,
    domain=False
)

st.altair_chart(chart, width="stretch")

# Loop
if st.session_state["running"]:
    time.sleep(0.05)
    st.rerun()
