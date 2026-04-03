# Pure Pursuit + FTG (F1TENTH)

This repo runs a 2-car F1TENTH simulation in Docker with RViz in noVNC.

## Quick Start

From repo root:

```bash
bash scripts/start_racetrack.sh
```

Open:

`http://localhost:8080/vnc.html`

Stop:

```bash
bash scripts/stop_racetrack.sh
```

Status:

```bash
bash scripts/status_racetrack.sh
```

## Where To Edit What

- To play with FTG behavior: `src/pure_pursuit/pure_pursuit/ftg_logic.py`
- To tune raceline tracking: `src/pure_pursuit/pure_pursuit/pure_pursuit_logic.py`
- To tune high-level control wiring (FTG vs track, overtake params): `src/pure_pursuit/pure_pursuit/controller_manager.py`
- To tune opponent detection + tracking: `src/state_machine/state_machine/opponent_detector.py`
- To tune state switching (`GB_TRACK` / `FTGONLY`): `src/state_machine/state_machine/state_machine.py`
- To change map and spawn points: `src/f1tenth_gym_ros/config/sim.yaml`
- To change launch parameters used in this project: `scripts/start_racetrack.sh`

## Questions

### Q1) What states are available in current state machine?

Only 2 states:
- `GB_TRACK`
- `FTGONLY`

File:
- `src/state_machine/state_machine/state_machine.py`

### Q2) When FTG mode is active, does pure pursuit stop?

Drive command is from FTG in `FTGONLY`.
Pure pursuit is still used as a small steering hint for FTG side preference.

File:
- `src/pure_pursuit/pure_pursuit/controller_manager.py`

### Q3) How is the other car detected?

`opponent_detector.py` uses LiDAR clusters and shape/range filters, then publishes:
- `/opponent_detection`
- `/opponent_detected`

File:
- `src/state_machine/state_machine/opponent_detector.py`

### Q4) How is tracking done?

Tracking is in `update_tracker(...)` inside `opponent_detector.py`.
It compares center changes across frames, smooths velocity, and requires persistence.

File:
- `src/state_machine/state_machine/opponent_detector.py`

### Q5) How is overtaking done?

Overtake is rule-based:
- detect opponent ahead and close
- choose left/right using open LiDAR side
- apply lateral target offset
- blend back to raceline

File:
- `src/pure_pursuit/pure_pursuit/controller_manager.py`

## Map Files

Main map path is set in:
- `src/f1tenth_gym_ros/config/sim.yaml`

Included map files:
- `src/f1tenth_gym_ros/maps/arc.pgm`
- `src/f1tenth_gym_ros/maps/arc.yaml`
- `src/f1tenth_gym_ros/maps/arc_obstacle.pgm`
- `src/f1tenth_gym_ros/maps/arc_obstacle.yaml`
