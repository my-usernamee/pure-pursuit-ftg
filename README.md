# F1TENTH Docker Workspace

ROS 2 + F1TENTH simulator workspace with a ready-to-run Docker/noVNC setup.
It launches the simulator, RViz, and the current autonomous driving stack
(`state_machine` + `pure_pursuit`) so you can tune behavior quickly.

## What Is Included

- Dockerized simulator (`f1tenth_gym_ros`)
- noVNC web client on `localhost:8080`
- Auto-launch scripts for start/stop/status
- Current driving logic in:
  - `src/pure_pursuit/`
  - `src/state_machine/`

## Prerequisites

- Docker Desktop (running)
- `docker compose` available
- macOS/Linux shell with `bash`

## Quick Start

From repository root:

```bash
bash scripts/start_racetrack.sh
```

Then open:

`http://localhost:8080/vnc.html`

Click **Connect** in noVNC. RViz and simulator should already be running.

## Tuning Workflow (RViz + Algorithm)

1. Keep the simulator running in Docker.
2. Edit tuning logic in:
   - `src/pure_pursuit/pure_pursuit/controller_manager.py`
   - `src/pure_pursuit/pure_pursuit/pure_pursuit_logic.py`
   - `src/pure_pursuit/pure_pursuit/ftg_logic.py`
   - `src/state_machine/state_machine/state_machine.py`
3. Re-run to apply changes:

```bash
bash scripts/stop_racetrack.sh
bash scripts/start_racetrack.sh
```

4. Watch live logs while tuning:

```bash
bash scripts/status_racetrack.sh
```

## Utility Commands

Start:

```bash
bash scripts/start_racetrack.sh
```

Status:

```bash
bash scripts/status_racetrack.sh
```

Stop:

```bash
bash scripts/stop_racetrack.sh
```

## GitHub Push Checklist

```bash
git add -A
git commit -m "cleanup repo and update docs"
git branch -M main
git remote add origin <your-github-repo-url>
git push -u origin main
```

If `origin` already exists, skip `git remote add origin ...`.

## Notes

- This repo is cleaned for source control: no colcon build outputs are tracked.
- The Docker compose mount is configured for this workspace layout (`../..:/sim_ws`).
