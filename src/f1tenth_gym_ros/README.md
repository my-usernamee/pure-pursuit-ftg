# f1tenth_gym_ros (Bridge Package)

This package bridges simulator state to ROS topics.

## Edit Here

- Map, topics, spawn points, agent count: `config/sim.yaml`
- Bridge publish/subscribe logic: `f1tenth_gym_ros/gym_bridge.py`
- Sim containers: `docker-compose.yml`
- RViz config: `launch/gym_bridge.rviz`

## Project Launch

Use root scripts:

```bash
bash scripts/start_racetrack.sh
bash scripts/status_racetrack.sh
bash scripts/stop_racetrack.sh
```
