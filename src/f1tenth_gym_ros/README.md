# f1tenth_gym_ros (Project Version)

This package is the simulator bridge used by this repo.
It connects the F1TENTH gym simulator to ROS topics used by your controller stack.

## What You Actually Need

Key files:
- `config/sim.yaml`: map path, topic names, spawn points, number of agents
- `f1tenth_gym_ros/gym_bridge.py`: publishes scans/odom/tf and receives drive commands
- `docker-compose.yml`: sim + noVNC services
- `launch/gym_bridge_launch.py`: starts bridge and RViz
- `launch/gym_bridge.rviz`: RViz config

## Topics Used In This Project

Published by bridge:
- `/scan`
- `/ego_racecar/odom`
- `/ego_racecar/opp_odom` (opponent odom in ego namespace)
- `/opp_scan`
- `/opp_racecar/odom`

Subscribed by bridge:
- `/drive` (ego drive)
- `/opp_drive` (opponent drive)

## Agent Count

Set in:
- `config/sim.yaml` -> `num_agent`

Current project uses 2 agents.

## Map

Set in:
- `config/sim.yaml` -> `map_path`

Map files used in this repo include:
- `maps/arc.pgm` / `maps/arc.yaml`
- `maps/arc_obstacle.pgm` / `maps/arc_obstacle.yaml`

## How It Is Started Here

Do not use old generic upstream instructions for this repo.
Use root scripts:

```bash
bash /Users/hari/Desktop/f1tenth_ws/scripts/start_racetrack.sh
```

Status:

```bash
bash /Users/hari/Desktop/f1tenth_ws/scripts/status_racetrack.sh
```

Stop:

```bash
bash /Users/hari/Desktop/f1tenth_ws/scripts/stop_racetrack.sh
```
