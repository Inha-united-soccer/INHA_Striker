import pygame
import time
import numpy as np
import config as CFG
import sim_logic as Logic
from db_manager import DBManager

# --- Constants ---
WIDTH, HEIGHT = 1400, 850  # Compact Layout
FIELD_WIDTH_PX = 600
FIELD_HEIGHT_PX = 400
FPS = 60

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREEN = (34, 139, 34)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
CYAN = (0, 255, 255)
ORANGE = (255, 165, 0)
YELLOW = (255, 255, 0)
DARK_RED = (139, 0, 0)
GRAY = (100, 100, 100)
LIGHT_GRAY = (200, 200, 200)

# --- Fallback Font (Minimal 3x5 or 4x6 for legibility without font module) ---
# 1=Draw, 0=Skip. 3 wide, 5 high.
FONT_MAP = {
    'A': [0,1,0, 1,0,1, 1,1,1, 1,0,1, 1,0,1],
    'B': [1,1,0, 1,0,1, 1,1,0, 1,0,1, 1,1,0],
    'C': [0,1,1, 1,0,0, 1,0,0, 1,0,0, 0,1,1],
    'D': [1,1,0, 1,0,1, 1,0,1, 1,0,1, 1,1,0],
    'E': [1,1,1, 1,0,0, 1,1,0, 1,0,0, 1,1,1],
    'F': [1,1,1, 1,0,0, 1,1,0, 1,0,0, 1,0,0],
    'G': [0,1,1, 1,0,0, 1,0,1, 1,0,1, 0,1,1],
    'H': [1,0,1, 1,0,1, 1,1,1, 1,0,1, 1,0,1],
    'I': [1,1,1, 0,1,0, 0,1,0, 0,1,0, 1,1,1],
    'J': [0,0,1, 0,0,1, 0,0,1, 1,0,1, 0,1,0],
    'K': [1,0,1, 1,0,1, 1,1,0, 1,0,1, 1,0,1],
    'L': [1,0,0, 1,0,0, 1,0,0, 1,0,0, 1,1,1],
    'M': [1,0,1, 1,1,1, 1,0,1, 1,0,1, 1,0,1],
    'N': [1,1,0, 1,0,1, 1,0,1, 1,0,1, 1,0,1], 
    'O': [0,1,0, 1,0,1, 1,0,1, 1,0,1, 0,1,0],
    'P': [1,1,0, 1,0,1, 1,1,0, 1,0,0, 1,0,0],
    'Q': [0,1,0, 1,0,1, 1,0,1, 1,0,1, 0,1,1], # simplified
    'R': [1,1,0, 1,0,1, 1,1,0, 1,0,1, 1,0,1],
    'S': [0,1,1, 1,0,0, 0,1,0, 0,0,1, 1,1,0],
    'T': [1,1,1, 0,1,0, 0,1,0, 0,1,0, 0,1,0],
    'U': [1,0,1, 1,0,1, 1,0,1, 1,0,1, 0,1,0],
    'V': [1,0,1, 1,0,1, 1,0,1, 1,0,1, 0,1,0],
    'W': [1,0,1, 1,0,1, 1,0,1, 1,1,1, 1,0,1],
    'X': [1,0,1, 1,0,1, 0,1,0, 1,0,1, 1,0,1],
    'Y': [1,0,1, 1,0,1, 0,1,0, 0,1,0, 0,1,0],
    'Z': [1,1,1, 0,0,1, 0,1,0, 1,0,0, 1,1,1],
    '0': [0,1,0, 1,0,1, 1,0,1, 1,0,1, 0,1,0],
    '1': [0,1,0, 1,1,0, 0,1,0, 0,1,0, 1,1,1],
    '2': [1,1,0, 0,0,1, 0,1,0, 1,0,0, 1,1,1],
    '3': [1,1,0, 0,0,1, 0,1,0, 0,0,1, 1,1,0],
    '4': [1,0,1, 1,0,1, 1,1,1, 0,0,1, 0,0,1],
    '5': [1,1,1, 1,0,0, 1,1,0, 0,0,1, 1,1,0],
    '6': [0,1,1, 1,0,0, 1,1,0, 1,0,1, 0,1,0],
    '7': [1,1,1, 0,0,1, 0,1,0, 0,1,0, 0,1,0],
    '8': [0,1,0, 1,0,1, 0,1,0, 1,0,1, 0,1,0],
    '9': [0,1,0, 1,0,1, 0,1,1, 0,0,1, 0,1,0],
    '.': [0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,1,0],
    ':': [0,0,0, 0,1,0, 0,0,0, 0,1,0, 0,0,0],
    '-': [0,0,0, 0,0,0, 1,1,1, 0,0,0, 0,0,0],
    ' ': [0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0],
    '_': [0,0,0, 0,0,0, 0,0,0, 0,0,0, 1,1,1],
}

def draw_text_fallback(screen, text, x, y, color=WHITE, scale=2):
    """Draws text using the bitmap font map."""
    text = str(text).upper()
    cur_x = x
    for char in text:
        if char in FONT_MAP:
            data = FONT_MAP[char]
            # 3x5 grid
            for r in range(5):
                for c in range(3):
                    idx = r * 3 + c
                    if data[idx]:
                        pygame.draw.rect(screen, color, 
                                         (cur_x + c*scale, y + r*scale, scale, scale))
            cur_x += 4 * scale # 3 width + 1 spacing
        else:
            cur_x += 4 * scale

class VectorFont:
    def render(self, text, antialias, color):
        # Returns a surface with the text drawn manually
        scale = 2
        w = len(text) * 4 * scale
        h = 6 * scale # 5 rows + 1 for descenders/spacing
        surf = pygame.Surface((w, h), pygame.SRCALPHA)
        draw_text_fallback(surf, text, 0, 0, color, scale)
        return surf

class Slider:
    def __init__(self, x, y, w, h, min_val, max_val, initial_val, label, param_key, dict_ref):
        self.rect = pygame.Rect(x, y, w, h)
        self.min_val = min_val
        self.max_val = max_val
        self.val = initial_val
        self.label = label
        self.param_key = param_key
        self.dict_ref = dict_ref # Reference to params dict (st_params or pass_params)
        self.dragging = False
        
        # Handle
        self.handle_width = 10
        self.update_handle_pos()

    def update_handle_pos(self):
        ratio = (self.val - self.min_val) / (self.max_val - self.min_val)
        handle_x = self.rect.x + ratio * self.rect.width - self.handle_width // 2
        self.handle_rect = pygame.Rect(handle_x, self.rect.y - 5, self.handle_width, self.rect.height + 10)

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            if self.handle_rect.collidepoint(event.pos) or self.rect.collidepoint(event.pos):
                self.dragging = True
                self.update_val_from_pos(event.pos[0])
        elif event.type == pygame.MOUSEBUTTONUP:
            self.dragging = False
        elif event.type == pygame.MOUSEMOTION:
            if self.dragging:
                self.update_val_from_pos(event.pos[0])

    def update_val_from_pos(self, mouse_x):
        rel_x = mouse_x - self.rect.x
        rel_x = max(0, min(rel_x, self.rect.width))
        ratio = rel_x / self.rect.width
        self.val = self.min_val + ratio * (self.max_val - self.min_val)
        self.update_handle_pos()
        # Update Dictionary in Real-time
        self.dict_ref[self.param_key] = self.val

    def draw(self, screen, font):
        # Draw Label & Value
        if font:
            label_surf = font.render(f"{self.label}: {self.val:.2f}", True, WHITE)
            screen.blit(label_surf, (self.rect.x, self.rect.y - 17))
        
        # Draw Bar
        pygame.draw.rect(screen, GRAY, self.rect)
        # Draw Filled Part
        filled_w = self.handle_rect.centerx - self.rect.x
        pygame.draw.rect(screen, CYAN, (self.rect.x, self.rect.y, filled_w, self.rect.height))
        # Draw Handle
        pygame.draw.rect(screen, WHITE, self.handle_rect)

def compute_heatmap_surface(width, height, score_func, bounds, args):
    """
    Generates a heatmap surface with improved gradients and max peak marker.
    bounds: (min_x, max_x, min_y, max_y)
    """
    surf = pygame.Surface((width, height))
    pixels = pygame.PixelArray(surf)
    
    # Grid limits
    min_x, max_x, min_y, max_y = bounds
    
    step_x = (max_x - min_x) / width
    step_y = (max_y - min_y) / height
    
    local_max_score = -1e9
    local_max_pos = None

    # Gradient Stops: (Value, Color)
    def get_color(val):
        if val < 0:
            t = (val - (-10)) / 10.0
            t = max(0, min(1, t))
            return (0, int(255*t), 255) # Blue to Cyan
        elif val < 5:
            t = val / 5.0
            return (0, 255, int(255*(1-t))) # Cyan to Green
        elif val < 10:
            t = (val - 5) / 5.0
            return (int(255*t), 255, 0) # Green to Yellow
        else:
            t = (val - 10) / 5.0
            t = max(0, min(1, t))
            return (255, int(255*(1-t)), 0) # Yellow to Red

    for py in range(height):
        world_y = max_y - py * step_y
        for px in range(width):
            world_x = min_x + px * step_x
            score = score_func(world_x, world_y, *args)
            
            pixels[px, py] = get_color(score)
            
            if score > local_max_score:
                local_max_score = score
                local_max_pos = (px, py)
            
    del pixels
    
    # Draw Max Marker
    if local_max_pos:
        mx, my = local_max_pos
        pygame.draw.circle(surf, WHITE, (mx, my), 2)
        pygame.draw.line(surf, BLACK, (mx-3, my), (mx+3, my), 1)
        pygame.draw.line(surf, BLACK, (mx, my-3), (mx, my+3), 1)
        
    return surf

def world_to_screen(x, y):
    """Convert world coordinates (meters) to screen coordinates (pixels)."""
    # Centered in Field Rect which is (350, 425) center.
    
    scale = 600.0 / 9.0 # Fits 9m field into 600px width
    
    center_x = 350
    center_y = 425
    
    sx = center_x + int(x * scale)
    sy = center_y - int(y * scale)
    return sx, sy

def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Soccer Simulation (PyGame FHD)")
    clock = pygame.time.Clock()
    
    # Initialize Font with Vector Backup
    try:
        pygame.font.init()
        font = pygame.font.SysFont("Arial", 18)
    except Exception as e:
        print(f"Font init failed ({e}), using fallback.")
        font = None
        
    if font is None:
        font = VectorFont() # Use our manual bitmap font

    # Initialize Logic Components
    db = DBManager()
    
    # Game State
    running = True
    paused = False 
    recording = False
    rec_data = []
    rec_start_time = 0
    
    # Initial Positions from CONFIG
    ball = Logic.Pose2D(*CFG.INITIAL_POSITIONS["ball"])
    passer = Logic.Pose2D(*CFG.INITIAL_POSITIONS["passer"])
    striker = Logic.Pose2D(*CFG.INITIAL_POSITIONS["striker"])
    opp_user = Logic.Pose2D(*CFG.INITIAL_POSITIONS["opp_user"])
    
    # Define teammates/opponents lists if needed by Logic, or just rely on Logic.
    # Note: Teammates/Opponents are typically managed by 'sim_logic' or a list here.
    # If they are managed here:
    # teammates = [Logic.Teammate(pid, Logic.Pose2D(x, y)) for pid, x, y in CFG.INITIAL_TEAMMATES]
    # opponents = [Logic.Opponent(Logic.Pose2D(x, y)) for x, y in CFG.INITIAL_OPPONENTS]
    
    # Stats
    goals = 0
    fails = 0

    # Cached Params
    st_params = CFG.ST_PARAMS.copy()
    pass_params = CFG.PASS_PARAMS.copy()
    
    # Heatmap State
    heatmap_timer = 0
    striker_hm_surf = None
    pass_hm_surf = None
    
    # Heatmap Bounds (Live View) using defaults
    striker_bounds = (-5, 5, -3.5, 3.5)
    pass_bounds = (-5, 5, -3.5, 3.5)

    # --- Initialize Sliders (ALL PARAMS) ---
    sliders = []
    col1_x = 730
    col2_x = 1050
    sy = 320 # Start below heatmaps
    sw = 280
    sh = 10
    gap = 33 # Adjustment: -5px
    
    # Helper to add slider
    def add_s(col, idx, key, label, minv, maxv, ref):
        x = col1_x if col == 1 else col2_x
        sliders.append(Slider(x, sy + idx*gap, sw, sh, minv, maxv, ref[key], label, key, ref))

    # Column 1: Striker Weights
    c1 = 0
    add_s(1, c1, "base_x_weight", "Base X", 0.0, 20.0, st_params); c1+=1
    add_s(1, c1, "center_y_weight", "Center Y", 0.0, 10.0, st_params); c1+=1
    add_s(1, c1, "defender_dist_weight", "Def Dist W", 0.0, 50.0, st_params); c1+=1
    add_s(1, c1, "defender_dist_cap", "Def Dist Cap", 1.0, 10.0, st_params); c1+=1
    add_s(1, c1, "hysteresis_x_weight", "Hysteresis X", 0.0, 10.0, st_params); c1+=1
    add_s(1, c1, "hysteresis_y_weight", "Hysteresis Y", 0.0, 10.0, st_params); c1+=1
    add_s(1, c1, "symmetry_weight", "Symmetry W", 0.0, 30.0, st_params); c1+=1
    add_s(1, c1, "ball_dist_weight", "Ball Dist W", 0.0, 20.0, st_params); c1+=1
    add_s(1, c1, "forward_weight", "Forward Bias", 0.0, 20.0, st_params); c1+=1
    add_s(1, c1, "penalty_weight", "Gen Penalty", 0.0, 30.0, st_params); c1+=1
    add_s(1, c1, "path_margin", "Path Margin", 0.1, 5.0, st_params); c1+=1
    add_s(1, c1, "pass_penalty_weight", "Pass Path Pen", 0.0, 50.0, st_params); c1+=1
    add_s(1, c1, "shot_penalty_weight", "Shot Path Pen", 0.0, 50.0, st_params); c1+=1
    add_s(1, c1, "movement_penalty_weight", "Move Path Pen", 0.0, 50.0, st_params); c1+=1

    # Column 2: Pass Logic
    c2 = 0
    add_s(2, c2, "min_pass_threshold", "Min Pass Dist", 0.0, 5.0, pass_params); c2+=1
    add_s(2, c2, "max_pass_threshold", "Max Pass Dist", 2.0, 10.0, pass_params); c2+=1
    add_s(2, c2, "tm_select_w_dist", "TM Sel Dist", 0.0, 5.0, pass_params); c2+=1
    add_s(2, c2, "tm_select_w_x", "TM Sel X", 0.0, 5.0, pass_params); c2+=1
    add_s(2, c2, "base_score", "Pass Base Scr", 0.0, 20.0, pass_params); c2+=1
    add_s(2, c2, "w_abs_dx", "Pass Acc X", 0.0, 5.0, pass_params); c2+=1
    add_s(2, c2, "w_abs_dy", "Pass Acc Y", 0.0, 5.0, pass_params); c2+=1
    add_s(2, c2, "w_x", "Pass Fwd Bias", 0.0, 5.0, pass_params); c2+=1
    add_s(2, c2, "w_y", "Pass Cnt Bias", 0.0, 5.0, pass_params); c2+=1
    add_s(2, c2, "opp_penalty", "Pass Block Pen", 0.0, 50.0, pass_params); c2+=1
    add_s(2, c2, "score_threshold", "Pass Thresh", 0.0, 20.0, pass_params); c2+=1

    while running:
        # 1. Event Handling
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    paused = not paused
                elif event.key == pygame.K_r:
                    if not recording:
                        recording = True
                        rec_data = []
                        rec_start_time = time.time()
                        print("Recording Started")
                    else:
                        recording = False
                        duration = time.time() - rec_start_time
                        run_id = db.save_run(duration, rec_data, pass_params, st_params)
                        print(f"Run {run_id} Saved!")
                elif event.key == pygame.K_ESCAPE:
                    running = False
            
            # Slider Events
            for s in sliders:
                s.handle_event(event)

        # 2. Controls
        keys = pygame.key.get_pressed()
        step = CFG.SIM["user_step"]
        move_speed = 5.0 * (1.0/FPS) # m/s * dt
        
        if keys[pygame.K_w] or keys[pygame.K_UP]:    opp_user.y += move_speed
        if keys[pygame.K_s] or keys[pygame.K_DOWN]:  opp_user.y -= move_speed
        if keys[pygame.K_a] or keys[pygame.K_LEFT]:  opp_user.x -= move_speed
        if keys[pygame.K_d] or keys[pygame.K_RIGHT]: opp_user.x += move_speed

        opp_user.x = Logic.clamp(opp_user.x, -5.0, 5.0)
        opp_user.y = Logic.clamp(opp_user.y, -3.5, 3.5)

        # 3. Simulation Step
        best_pos = (0,0)
        best_score = 0.0
        pfound = False
        pass_target = Logic.Pose2D(0,0)
        best_tm_cache = None
        current_pass_score = 0.0

        # Calculate opponents list (Live)
        opponents = [Logic.Opponent(Logic.Pose2D(-3.8, 0.5), 0.0), Logic.Opponent(opp_user, 0.0)]
        
        if not paused:
            # AI Logic (Movement)
            best_pos, best_score = Logic.compute_striker_costmap(striker, ball, opponents, st_params)
            target = Logic.Pose2D(best_pos[0], best_pos[1])
            
            dt = 1.0 / FPS
            speed = 3.0
            striker = Logic.move_towards(striker, target, speed, dt)
            
            # Pass Logic
            teammates = [Logic.Teammate(1, passer), Logic.Teammate(2, striker)]
            best_tm = Logic.select_best_teammate(ball, teammates, 1, pass_params)
            best_tm_cache = best_tm
            if best_tm:
                ptx, pty, psc = Logic.compute_pass_costmap(ball, best_tm, opponents, pass_params)
                current_pass_score = psc
                if psc >= pass_params["score_threshold"]:
                    pfound = True
                    pass_target = Logic.Pose2D(ptx, pty)

            # Logging
            if recording:
                rec_data.append({
                    "t": time.time(),
                    "ofb_score": float(best_score),
                    "striker": (float(striker.x), float(striker.y))
                })
        else:
            # Still compute best pos for vis
            best_pos, best_score = Logic.compute_striker_costmap(striker, ball, opponents, st_params)
            # Hypothetical pass optimization for viz
            teammates = [Logic.Teammate(1, passer), Logic.Teammate(2, striker)]
            best_tm_cache = Logic.select_best_teammate(ball, teammates, 1, pass_params)

        # --- Update Heatmaps (Throttled but more frequent for smoothness) ---
        heatmap_timer += 1
        if heatmap_timer % 5 == 0: 
            # Wrapper for Striker Score
            def s_score(x, y, robot, ball, opps, params):
                return Logic.compute_striker_score(x, y, robot, ball, opps, params)
            
            # Local Bounds: Striker +/- 3.0m
            sb_min_x = max(-5.0, striker.x - 3.0)
            sb_max_x = min(5.0, striker.x + 3.0)
            sb_min_y = max(-3.5, striker.y - 2.5)
            sb_max_y = min(3.5, striker.y + 2.5)
            striker_bounds = (sb_min_x, sb_max_x, sb_min_y, sb_max_y)

            striker_hm_surf = compute_heatmap_surface(
                100, 70, # Low Res
                s_score,
                striker_bounds,
                (striker, ball, opponents, st_params)
            )
            
            # Pass Bounds: Receiver +/- 3.0m
            if best_tm_cache:
                ref_x, ref_y = best_tm_cache.pos.x, best_tm_cache.pos.y
            else:
                ref_x, ref_y = ball.x, ball.y # Fallback
                
            pb_min_x = max(-5.0, ref_x - 3.0)
            pb_max_x = min(5.0, ref_x + 3.0)
            pb_min_y = max(-3.5, ref_y - 2.5)
            pb_max_y = min(3.5, ref_y + 2.5)
            pass_bounds = (pb_min_x, pb_max_x, pb_min_y, pb_max_y)
            
            def p_score(x, y, ball, opps, params):
                 # Enforce Pass Distance Constraints
                 d = ((x - ball.x)**2 + (y - ball.y)**2)**0.5
                 if d < params["min_pass_threshold"] or d > params["max_pass_threshold"]:
                     return -20.0
                 
                 tm = Logic.Teammate(99, Logic.Pose2D(x, y))
                 return Logic.compute_pass_score_for_target(ball, tm, x, y, opps, params)
                 
            pass_hm_surf = compute_heatmap_surface(
                100, 70,
                p_score,
                pass_bounds,
                (ball, opponents, pass_params)
            )

        # 4. Rendering
        screen.fill(BLACK)
        
        # --- Helper for Rulers ---
        def draw_ruler(surf, rect, bounds, color=WHITE):
            min_x, max_x, min_y, max_y = bounds
            w, h = rect.width, rect.height
            
            # Draw Border
            pygame.draw.rect(surf, color, rect, 1)
            
            # Center Cross
            cx = rect.x + (0 - min_x)/(max_x-min_x) * w
            cy = rect.y + (max_y - 0)/(max_y-min_y) * h
            if 0 <= cx <= rect.right and 0 <= cy <= rect.bottom:
                 pygame.draw.line(surf, GRAY, (cx, rect.top), (cx, rect.bottom), 1)
                 pygame.draw.line(surf, GRAY, (rect.left, cy), (rect.right, cy), 1)

            # Labels (corners)
            if font:
                # TL
                lbl = font.render(f"{min_x:.1f},{max_y:.1f}", True, color)
                surf.blit(lbl, (rect.x + 2, rect.y + 2))
                # BR
                lbl = font.render(f"{max_x:.1f},{min_y:.1f}", True, color)
                surf.blit(lbl, (rect.right - lbl.get_width() - 2, rect.bottom - lbl.get_height() - 2))

        # --- Main Field (Centered at 350, 425) ---
        field_rect = pygame.Rect(
            350 - FIELD_WIDTH_PX//2, 
            425 - FIELD_HEIGHT_PX//2, 
            FIELD_WIDTH_PX, 
            FIELD_HEIGHT_PX
        )
        pygame.draw.rect(screen, GREEN, field_rect)
        pygame.draw.rect(screen, WHITE, field_rect, 2)
        pygame.draw.line(screen, WHITE, 
                         (350, 425 - FIELD_HEIGHT_PX//2), 
                         (350, 425 + FIELD_HEIGHT_PX//2), 2)
        
        # Draw Rulers on Field
        draw_ruler(screen, field_rect, (-5.0, 5.0, -3.5, 3.5), LIGHT_GRAY)

        # Draw Bounds Visualization on Field
        def draw_bounds_rect(bounds, color):
            mx, Mx, my, My = bounds
            # Convert to screen coords
            x1, y1 = world_to_screen(mx, My) # TL
            x2, y2 = world_to_screen(Mx, my) # BR
            r = pygame.Rect(x1, y1, x2-x1, y2-y1)
            pygame.draw.rect(screen, color, r, 2)
            
        draw_bounds_rect(striker_bounds, CYAN)
        draw_bounds_rect(pass_bounds, YELLOW)

        # Draw Goals
        g_width, g_depth = 40, 120 
        g_left = field_rect.left - 40 
        g_cy = 425
        pygame.draw.rect(screen, GREEN, (g_left, g_cy - g_depth//2, 40, g_depth), 1)
        pygame.draw.rect(screen, GREEN, (field_rect.right, g_cy - g_depth//2, 40, g_depth), 1)

        # Entities Drawing
        def draw_entity(pose, color, radius=10, shape="circle"):
            sx, sy = world_to_screen(pose.x, pose.y)
            if shape == "circle":
                pygame.draw.circle(screen, color, (sx, sy), radius)
            elif shape == "rect":
                pygame.draw.rect(screen, color, (sx-radius, sy-radius, radius*2, radius*2))
            elif shape == "diamond":
                points = [(sx, sy-radius), (sx+radius, sy), (sx, sy+radius), (sx-radius, sy)]
                pygame.draw.polygon(screen, color, points)

        draw_entity(striker, CYAN, 12, "circle") 
        draw_entity(passer, BLUE, 12, "rect")
        draw_entity(ball, ORANGE, 8, "circle")
        draw_entity(opp_user, RED, 14, "diamond")
        draw_entity(Logic.Pose2D(-3.8, 0.5), DARK_RED, 12, "rect") # GK

        if not paused and best_pos:
            tx, ty = world_to_screen(best_pos[0], best_pos[1])
            pygame.draw.line(screen, WHITE, (tx-5, ty), (tx+5, ty), 1)
            pygame.draw.line(screen, WHITE, (tx, ty-5), (tx, ty+5), 1)
            
        if pfound:
            tx, ty = world_to_screen(pass_target.x, pass_target.y)
            pygame.draw.circle(screen, YELLOW, (tx, ty), 5)
            px, py = world_to_screen(ball.x, ball.y)
            pygame.draw.line(screen, YELLOW, (px, py), (tx, ty), 1)

        # --- Right Panel (UI) ---
        panel_x = 700
        pygame.draw.rect(screen, (30, 30, 30), (panel_x, 0, WIDTH-panel_x, HEIGHT))
        pygame.draw.line(screen, WHITE, (panel_x, 0), (panel_x, HEIGHT))
        
        # Headers
        if font:
            screen.blit(font.render("Striker Optimization", True, WHITE), (730, 10))
            screen.blit(font.render("Pass Decision", True, WHITE), (1050, 10))

        # Draw Heatmaps
        hm_w, hm_h = 300, 210
        if striker_hm_surf:
            scaled_hm = pygame.transform.scale(striker_hm_surf, (hm_w, hm_h))
            screen.blit(scaled_hm, (730, 30))
            r = pygame.Rect(730, 30, hm_w, hm_h)
            draw_ruler(screen, r, striker_bounds, CYAN)
            
        if pass_hm_surf:
             scaled_hm = pygame.transform.scale(pass_hm_surf, (hm_w, hm_h))
             screen.blit(scaled_hm, (1050, 30))
             r = pygame.Rect(1050, 30, hm_w, hm_h)
             draw_ruler(screen, r, pass_bounds, YELLOW)

        # Draw Sliders Headers
        if font:
            screen.blit(font.render("Optimization Weights", True, YELLOW), (730, 280))
            screen.blit(font.render("Pass Logic Params", True, YELLOW), (1050, 280))

        for s in sliders:
            s.draw(screen, font)

        # Pause Overlay
        if paused:
            cx, cy = 350, 425
            pygame.draw.rect(screen, WHITE, (cx - 20, cy - 30, 10, 60))
            pygame.draw.rect(screen, WHITE, (cx + 10, cy - 30, 10, 60))
            pygame.draw.rect(screen, RED, (350 - 300, 425 - 200, 600, 400), 5)

        # FPS overlay
        if font:
             screen.blit(font.render(f"FPS: {clock.get_fps():.1f}", True, WHITE), (10, 10))
             screen.blit(font.render(f"OFB Score: {best_score:.2f}", True, WHITE), (10, 35))
             screen.blit(font.render(f"Pass Score: {current_pass_score:.2f}", True, WHITE), (10, 60))
             
             if recording:
                 screen.blit(font.render("● RECORDING", True, RED), (10, 85))

        pygame.display.flip()
        clock.tick(FPS)
    
    pygame.quit()

if __name__ == "__main__":
    main()
