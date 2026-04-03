# f1tenth_gym_ros Package

Use this package for simulator bridge + map/topic setup.

Exact files to tweak:
- Map path, spawn points, agent count, topic names: `config/sim.yaml`
- Bridge publish/subscribe behavior: `f1tenth_gym_ros/gym_bridge.py`
- Docker services for sim/noVNC: `docker-compose.yml`
- Launch file: `launch/gym_bridge_launch.py`
- RViz layout: `launch/gym_bridge.rviz`
