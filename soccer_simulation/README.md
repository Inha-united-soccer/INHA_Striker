# Soccer Game Simulation

This is a PyGame-based soccer simulation that logs performance data to a central database.

## Installation

1. **Clone the repository:**
   ```bash
   git clone https://github.com/Inha-United/Soccer.git
   cd Soccer/soccer_game
   ```

2. **Setup Virtual Environment:**
   Run the setup script (Mac/Linux):
   ```bash
   ./start_game.sh
   ```
   *Note: If `start_game.sh` fails, you may need to install dependencies manually:*
   ```bash
   python3 -m venv ../.venv
   source ../.venv/bin/activate
   pip install -r requirements.txt
   ```

## Database Setup

**The database key is included in this repository for your convenience.** 
It is pre-configured to log data to the central server. No setup required!

*(Note: In a production app, we would hide this key, but for this project, we've included it to make it easy to run.)*

## Running the Game

Simply run:
```bash
./start_game.sh
```

## Controls
- **WASD / Arrows**: Move the opponent (Red)
- **Space**: Pause/Resume
- **R**: Start/Stop Recording (Videos are saved and data uploaded to DB)
- **Sliders**: Adjust AI parameters in real-time
