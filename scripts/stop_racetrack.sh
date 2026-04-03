#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
COMPOSE_DIR="${ROOT_DIR}/src/f1tenth_gym_ros"

cd "${COMPOSE_DIR}"
SIM_CONTAINER_ID="$(docker compose ps -q sim || true)"

if [[ -n "${SIM_CONTAINER_ID}" ]]; then
  echo "Stopping controller and sim processes in container..."
  docker exec "${SIM_CONTAINER_ID}" bash -lc \
    "for PID_FILE in \
      /tmp/f1tenth_controller_manager.pid \
      /tmp/f1tenth_opp_controller_manager.pid \
      /tmp/f1tenth_state_machine.pid \
      /tmp/f1tenth_opponent_detector.pid \
      /tmp/f1tenth_autodrive.pid \
      /tmp/f1tenth_launch.pid; do \
      if [[ -f \"\${PID_FILE}\" ]]; then \
        PID=\$(cat \"\${PID_FILE}\"); \
        if ps -p \"\${PID}\" >/dev/null 2>&1; then kill \"\${PID}\" || true; fi; \
        rm -f \"\${PID_FILE}\"; \
      fi; \
    done"
fi

echo "Stopping Docker services..."
docker compose down

echo "Race track stopped."
