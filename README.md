# Pure Pursuit + FTG (F1TENTH)

This repo runs a 2-car F1TENTH simulation in Docker with RViz in noVNC.

Use this if you want:
- car running in browser (`localhost:8080`)
- pure pursuit on raceline in normal mode
- FTG for close obstacle avoidance
- simple overtake behavior against opponent car

## Quick Start

From repo root:

```bash
bash scripts/start_racetrack.sh
```

Open:

`http://localhost:8080/vnc.html`

To stop:

```bash
bash scripts/stop_racetrack.sh
```

To check status/logs:

```bash
bash scripts/status_racetrack.sh
```

## Exact Code Map

Main runtime files:
- `scripts/start_racetrack.sh`: starts sim, state machine, opponent detector, and both controllers
- `src/state_machine/state_machine/state_machine.py`: state switching (`GB_TRACK` / `FTGONLY`)
- `src/state_machine/state_machine/opponent_detector.py`: opponent detection + tracking over time
- `src/state_machine/state_machine/obstacle_detector.py`: simple front obstacle alarm (not tracker)
- `src/pure_pursuit/pure_pursuit/controller_manager.py`: pure pursuit control, FTG dispatch, overtake logic
- `src/pure_pursuit/pure_pursuit/ftg_logic.py`: follow-the-gap implementation
- `src/pure_pursuit/pure_pursuit/pure_pursuit_logic.py`: raceline target point + steering math
- `src/f1tenth_gym_ros/config/sim.yaml`: map, spawn points, topic names, number of agents
- `src/f1tenth_gym_ros/f1tenth_gym_ros/gym_bridge.py`: simulator-to-ROS bridge (publishes scans/odom)

## Answers To Your Questions

### 1) What states are available in current state machine?

Only 2 states right now:
- `GB_TRACK`: track-follow mode
- `FTGONLY`: avoid mode (follow-the-gap)

File:
- `src/state_machine/state_machine/state_machine.py`

There is no separate `TRAIL` state and no separate `OVERTAKE` state in the state machine.

### 2) When FTG mode is active, does pure pursuit stop?

Drive commands come from FTG in `FTGONLY`.

What still happens:
- pure pursuit is used only to compute a steering hint so FTG can prefer a useful side.

File:
- `src/pure_pursuit/pure_pursuit/controller_manager.py`

### 3) How is the other car detected?

`opponent_detector.py` uses LiDAR and does:
- make point clusters
- keep clusters with car-like size/range
- optional filter: keep detections near raceline
- publish `/opponent_detection` and `/opponent_detected`

File:
- `src/state_machine/state_machine/opponent_detector.py`

### 4) How is tracking done?

Tracking is also in `opponent_detector.py`:
- compares current center vs previous center
- computes relative velocity estimate
- smooths it
- requires persistence over a few frames
- briefly holds last valid detection to avoid flicker

Function location:
- `update_tracker(...)` in `src/state_machine/state_machine/opponent_detector.py`

### 5) Which code tracks obstacle in this repo?

Real temporal tracking is in:
- `src/state_machine/state_machine/opponent_detector.py` (`update_tracker`)

`obstacle_detector.py` is only a simple distance check in front window, not a temporal tracker.

### 6) What is `obstacle_detector.py` vs `opponent_detector.py`?

- `obstacle_detector.py`: simple "something is close in front" detector
- `opponent_detector.py`: car-specific detector + tracker, used for opponent-aware behavior

Current launch uses `opponent_detector.py`.

File that launches nodes:
- `scripts/start_racetrack.sh`

### 7) How is overtaking done?

Overtake is not a full local planner.

Current behavior:
- normal base path is pure pursuit on raceline
- if opponent is ahead and close, pick left/right based on open LiDAR side
- shift target point sideways by fixed offset
- blend back to raceline after clear

File:
- `src/pure_pursuit/pure_pursuit/controller_manager.py`

So yes: this is rule-based (heuristic) overtake, not MPC/RRT/etc.

### 8) If opponent does not move in same circular motion, will overtake still work?

Sometimes yes, but less reliable.

Why:
- detection may ignore targets too far from raceline (track filter)
- overtake uses fixed side offset, not a dynamic trajectory optimizer

Files:
- `src/state_machine/state_machine/opponent_detector.py`
- `src/pure_pursuit/pure_pursuit/controller_manager.py`

## Map Files

Main map currently used by bridge:
- `src/f1tenth_gym_ros/config/sim.yaml` (`map_path`)

Your additional obstacle map files are included:
- `src/f1tenth_gym_ros/maps/arc_obstacle.pgm`
- `src/f1tenth_gym_ros/maps/arc_obstacle.yaml`

## What Is Not Part Of Current Main Pipeline

Legacy wall-follow lab code is not part of current run path:
- `src/gap_finder/wall_follow/`

Current stack is pure pursuit + FTG + opponent detector/state machine launched by:
- `scripts/start_racetrack.sh`
