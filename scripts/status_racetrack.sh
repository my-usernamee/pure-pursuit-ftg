#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
COMPOSE_DIR="${ROOT_DIR}/src/f1tenth_gym_ros"

cd "${COMPOSE_DIR}"
echo "Docker services:"
docker compose ps

SIM_CONTAINER_ID="$(docker compose ps -q sim || true)"
if [[ -n "${SIM_CONTAINER_ID}" ]]; then
  echo ""
  echo "ROS nodes:"
  docker exec "${SIM_CONTAINER_ID}" bash -lc \
    "source /opt/ros/foxy/setup.bash && source /sim_ws/install/local_setup.bash && timeout 8 ros2 node list" || true

  echo ""
  echo "Controller processes:"
  docker exec "${SIM_CONTAINER_ID}" bash -lc \
    "for NAME in \
      f1tenth_launch \
      f1tenth_state_machine \
      f1tenth_controller_manager \
      f1tenth_opp_controller_manager \
      f1tenth_opponent_detector \
      f1tenth_obstacle_detector; do \
      PID_FILE=\"/tmp/\${NAME}.pid\"; \
      if [[ -f \"\${PID_FILE}\" ]]; then \
        PID=\$(cat \"\${PID_FILE}\"); \
        if ps -p \"\${PID}\" >/dev/null 2>&1; then \
          echo \"\${NAME}: running (pid \${PID})\"; \
        else \
          echo \"\${NAME}: stale pid file\"; \
        fi; \
      else \
        echo \"\${NAME}: not running\"; \
      fi; \
    done"
fi

echo ""
echo "noVNC endpoint test:"
curl -I --max-time 5 http://localhost:8080/vnc.html | head -n 1
