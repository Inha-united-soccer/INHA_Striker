<div align="center">

# :soccer: INHA Striker
**Advanced Autonomous Agent for Humanoid Soccer**

[![ROS2](https://img.shields.io/badge/ROS2-Humble-3490dc.svg?style=for-the-badge&logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![C++](https://img.shields.io/badge/C++-17-00599C.svg?style=for-the-badge&logo=c%2B%2B&logoColor=white)](https://en.cppreference.com/w/cpp/17)
[![BehaviorTree](https://img.shields.io/badge/BehaviorTree-V4-2ca02c.svg?style=for-the-badge)](https://www.behaviortree.dev/)
[![License](https://img.shields.io/badge/License-Apache_2.0-yellow.svg?style=for-the-badge)](LICENSE)

*Dynamic Decision Making • Tactical Positioning • Human-like Agility*

---
</div>

## Striker Vision
**"To create a soccer-playing intelligence that doesn't just calculate, but *understands* the flow of the game."**

The **INHA Striker** is designed to bridge the gap between rigid robotic control and dynamic human intuition. By leveraging hierarchical behavior trees and advanced motion planning, our agent demonstrates adaptive gameplay—switching seamlessly between aggressive dribbling, tactical off-ball movement, and precision finishing.

---

## Key Feature

### **Hyper-Modular Architecture**
We separate **Strategic Intent** from **Mechanical Execution** using a novel **Parameter-Injection Pattern**.
*   **Strategy Layer**: A high-level director determines the mode (Attack, Defend, Time-Wasting).
*   **Tactics Layer (The Tuner)**: Instead of hard-coding behaviors, tactics simply **inject parameters** into the Blackboard.
    *   *Example*: "Pressing" tactic injects `speed_limit=1.0` and `kick_aggressiveness=High`.
    *   *Example*: "Tempo Control" tactic injects `speed_limit=0.4` and `kick_aggressiveness=Low`.
*   **Execution Layer (The Engine)**: The robust `StrikerDecision` node consumes these parameters to execute the optimal action without code changes.
    
**Benefit**: You can completely change the robot's playstyle by tweaking a few numbers in the Tactics layer, with zero risk of breaking the core movement logic.

### **Source Code Structure**
You can verify this modular architecture in our source tree:

*   📂 **[`src/brain/src/`](src/brain/src)**
    *   📂 **[`strategy/`](src/brain/src/strategy)** (Layer 1: Strategy Director)
        *   [`strategy_director.cpp`](src/brain/src/strategy/strategy_director.cpp)
        *   [`game_state_manager.cpp`](src/brain/src/strategy/game_state_manager.cpp)
        *   [`strategy_nodes.cpp`](src/brain/src/strategy/strategy_nodes.cpp)
    *   📂 **[`tactics/`](src/brain/src/tactics)** (Layer 2: Tactics / Tuners)
        *   [`tactic_selector.cpp`](src/brain/src/tactics/tactic_selector.cpp)
        *   [`tactics_definitions.cpp`](src/brain/src/tactics/tactics_definitions.cpp)
        *   [`tactics_nodes.cpp`](src/brain/src/tactics/tactics_nodes.cpp)
    *   ⚙️ **Layer 3: Execution Engines** (Consumers)
        *   [`striker_decision.cpp`](src/brain/src/striker_decision.cpp) (Main Decision Logic)
        *   [`offtheball.cpp`](src/brain/src/offtheball.cpp) (Movement Logic)
        *   [`chase.cpp`](src/brain/src/chase.cpp) 
        *   *... and more (kick, adjust, etc.)*
          
---

## Striker Behavior Tree Overview
[<img width="1000" height="430" alt="image" src="https://github.com/user-attachments/assets/f4c2592e-dc3f-4957-b1f5-4a4410184ee0" />](https://files.slack.com/files-pri/T0908EY4K6V-F0A9TF95XHD/striker.png)


---

## Contribution
This project contributes to the field of humanoid robotics by:
1.  **Demonstrating Robust Autonomy**: Showing how behavior trees can handle the chaotic environment of a soccer match.
2.  **Implementing Human-inspired Motion**: Proving that curvilinear paths are superior to linear point-to-point navigation for bipedal robots.
3.  **Open Source Innovation**: Providing a modular, extensible C++ framework for future researchers in the RoboCup domain.

---

<div align="center">
    <b>Built with by INHA United</b><br>
    <i>Pushing the boundaries of Autonomous Soccer</i>
</div>
