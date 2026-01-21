# AI Soccer Striker Agent

> **Advanced Autonomous Decision Making & Control System for Humanoid Soccer Robots**

This module implements the **Striker** intelligence for our humanoid soccer robot, designed to execute dynamic tactical behaviors in real-time. It features a robust **Behavior Tree (BT)** architecture integrated with advanced motion planning algorithms.

---

## Core Architecture

The Striker's cognitive process is structured into a hierarchical **Finite State Machine (FSM)** driven by a Behavior Tree, ensuring reactive yet deliberate actions.

| State | Description | Key Algorithm |
|-------|-------------|---------------|
| **Search** | Active perception to localize the ball. | `CamScanField` (Sine-wave Scanning) |
| **Co-op** | Positioning for passes & tactical support. | `OffTheBall` (Grid-based Utility Maximization) |
| **Approach** | Dynamic movement to ball or goal. | `Chase` (Curvilinear Swirl) / `Dribble` (Path Projection) |
| **Attack** | Precision alignment and shooting. | `Adjust` (P-Control) & `Kick` (Lock Mechanism) |

---

## Key Algorithms

### 1. Obstacle-Aware Dribble Planner
*   **Problem**: How to navigate through defenders while maintaining ball control?
*   **Solution**: **Trajectory Projection & Cost Optimization**
    *   Generates candidate paths towards the goal line.
    *   **Projects** obstacles onto each path to calculate clearance.
    *   Evaluates paths using a cost function: $J = w_c \cdot Clearance - w_d \cdot CenterPenalty + w_g \cdot GoalBonus$.
    *   Selects the optimal trajectory that balances safety (avoidance) and aggression (goal-scoring).

### 2. Dynamic Curvilinear Approach (Chase)
*   **Problem**: Approaching the ball directly often leads to poor kicking angles.
*   **Solution**: **Circle-Back with Swirl Control**
    *   Instead of a straight line, the robot calculates a **curvilinear path** to approach the ball from the rear (Goal-Ball-Robot alignment).
    *   **Swirl Logic**: Adds a tangential velocity vector when circling, creating a spiral motion that aligns the robot's heading faster while maintaining momentum.
    *   **Hysteresis Stabilization**: Uses dual thresholds (Enter 25° / Exit 15°) to prevent state oscillation between "Align" and "Push" phases.

### 3. Grid-based Tactical Positioning (Off-Ball)
*   **Problem**: Where should the robot stand when not in possession?
*   **Solution**: **Symmetry-based Grid Search**
    *   Discretizes the field into a grid.
    *   Calculates the **Centroid of Defenders** and identifies the **Symmetric Target Point** to exploit open space.
    *   Maximizes utility by avoiding defenders ($D < 3m$) and maintaining optimal distance from the goal.

### 4. Robust Kick Execution
*   **Kick Lock Mechanism**: To solve decision chattering (flickering between "Ready" and "Not Ready"), a temporal lock is applied once shooting conditions are met, ensuring the kick action completes.
*   **Adaptive Precision**:
    *   **Quick Kick**: Relaxed tolerances for set-pieces or close-range scrambles.
    *   **Precision Kick**: Strict alignment requirements for long-range shots.

---

## Active Perception

*   **Smooth Tracking**: Uses **EMA (Exponential Moving Average)** filtering on head joints to prevent jitter during tracking.
*   **Adaptive Gain**: Lowers P-gain and increases damping when tracking noisy ball detection (`CamTrackBall`), while using clean sine-wave trajectories for field scanning (`CamScanField`).

---

## Tech Stack
*   **Language**: C++17
*   **Framework**: BehaviorTree.CPP, ROS2 (Humble)
*   **Optimization**: Eigen 3 (Matrix operations)
*   **Visualization**: Rerun SDK (Real-time debugging)

---
*Created by [Your Name/Team Name] - Inha United*
