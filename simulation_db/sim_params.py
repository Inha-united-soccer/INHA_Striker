# Parameters for Soccer Simulation
# Users can modify these values to optimize Off-the-Ball and Pass logic.

# ============================================================
# 1) Pass Logic Parameters (CalcPassDir)
# ============================================================
PASS_PARAMS = {
    # Teammate Selection
    "min_pass_threshold": 1.0,  # Minimum distance to pass
    "max_pass_threshold": 4.0,  # Maximum distance to pass
    "tm_select_w_dist": 1.0,    # Penalty weight for distance (closer is better)
    "tm_select_w_x": 1.2,       # Penalty weight for X position (forward is better)

    # Field Dimensions (Half Court for calculation)
    "field_half_length": 4.5,
    "field_half_width": 3.0,

    # Grid Search Parameters
    "grid_half_xrange": 3.0,    # Search radius X
    "grid_half_yrange": 2.5,    # Search radius Y
    "grid_step": 0.2,           # Grid resolution (lower = more precise but slower)

    # Base Score Weights
    "base_score": 10.0,
    "w_abs_dx": 1.1,            # Packet dist pen X
    "w_abs_dy": 0.7,            # Packet dist pen Y
    "w_x": 0.95,                # Forward bias
    "w_y": 0.45,                # Center bias

    # Opponent Avoidance
    "opp_path_margin": 1.15,    # Safety margin around pass path
    "opp_penalty": 24.0,        # Score penalty for blocked path
    "opp_memory_sec": 5.0,      # How long to remember opponents

    # Threshold
    "score_threshold": 6.5,     # Minimum score to attempt pass
}

# ============================================================
# 2) Striker Off-the-Ball Parameters
# ============================================================
ST_PARAMS = {
    "field_length": 9.0,
    "field_width": 6.0,
    
    # Goal/Penalty Area dims
    "penalty_dist": 1.5,
    "goal_width": 2.0,
    "circle_radius": 0.75,
    "penalty_area_length": 2.0,
    "penalty_area_width": 5.0,
    "goal_area_length": 1.0,
    "goal_area_width": 3.0,

    # Positioning Goals
    "dist_from_goal": 2.0,      # Target distance from opponent goal

    # Weights (Cost Function)
    "base_x_weight": 5.0,       # Try to stay at 'dist_from_goal'
    "center_y_weight": 3.0,     # Try to stay central
    "defender_dist_weight": 20.0, # Avoid defenders (Weight)
    "defender_dist_cap": 3.0,   # Max distance to consider for avoidance
    
    "hysteresis_x_weight": 3.0, # Resist moving X (Stability)
    "hysteresis_y_weight": 3.0, # Resist moving Y (Stability)

    "symmetry_weight": 10.0,    # Stay symmetric to defenders?
    "ball_dist_weight": 3.0,    # Maintain specific distance from ball
    "forward_weight": 5.2,      # Biased towards opponent goal

    # Penalties
    "penalty_weight": 10.0,     # General penalty weight
    "path_margin": 1.5,         # Margin for shot/pass paths
    "opp_memory_sec": 5.0,      

    "pass_penalty_weight": 15.0,     # Penalty if pass path blocked
    "shot_penalty_weight": 3.0,      # Penalty if shot path blocked
    "movement_penalty_weight": 30.0, # Penalty if movement path blocked

    # Search Grid
    "path_confidence": 0.5,
    "search_x_margin": 1.7,
    "grid_step": 0.1,
}

# ============================================================
# 3) Simulation Settings
# ============================================================
SIM = {
    "dt": 0.1,                 # Time step (seconds)
    "player_speed": 0.5,       # Robot speed (m/s)
    "user_step": 0.05,         # Manual control step size
}
