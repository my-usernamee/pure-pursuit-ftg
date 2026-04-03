#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
COMPOSE_DIR="${ROOT_DIR}/src/f1tenth_gym_ros"

echo "[1/6] Starting Docker services..."
cd "${COMPOSE_DIR}"
docker compose up -d --force-recreate

SIM_CONTAINER_ID="$(docker compose ps -q sim)"
if [[ -z "${SIM_CONTAINER_ID}" ]]; then
  echo "Could not find running sim container."
  exit 1
fi

echo "[2/6] Building required ROS packages in sim container..."
docker exec "${SIM_CONTAINER_ID}" bash -lc \
  "source /opt/ros/foxy/setup.bash && \
   cd /sim_ws && \
   colcon build --symlink-install \
     --packages-select f1tenth_gym_ros pure_pursuit state_machine"

echo "[3/6] Stopping previous driving processes..."
docker exec "${SIM_CONTAINER_ID}" bash -lc \
  "for PID_FILE in \
      /tmp/f1tenth_controller_manager.pid \
      /tmp/f1tenth_opp_controller_manager.pid \
      /tmp/f1tenth_state_machine.pid \
      /tmp/f1tenth_opponent_detector.pid \
      /tmp/f1tenth_obstacle_detector.pid \
      /tmp/f1tenth_autodrive.pid \
      /tmp/f1tenth_launch.pid; do \
      if [[ -f \"\${PID_FILE}\" ]]; then \
        PID=\$(cat \"\${PID_FILE}\"); \
        if ps -p \"\${PID}\" >/dev/null 2>&1; then kill \"\${PID}\" || true; fi; \
        rm -f \"\${PID_FILE}\"; \
      fi; \
    done"

echo "[4/6] Starting simulator launch..."
docker exec -d "${SIM_CONTAINER_ID}" bash -lc \
  "source /opt/ros/foxy/setup.bash && \
   source /sim_ws/install/local_setup.bash && \
   nohup ros2 launch f1tenth_gym_ros gym_bridge_launch.py > /tmp/f1tenth_gym_launch.log 2>&1 & \
   echo \$! > /tmp/f1tenth_launch.pid"

echo "[5/6] Starting controller stack (GB_TRACK/FTGONLY)..."
sleep 2
docker exec -d "${SIM_CONTAINER_ID}" bash -lc \
  "source /opt/ros/foxy/setup.bash && \
   source /sim_ws/install/local_setup.bash && \
   nohup ros2 run state_machine state_machine \
     --ros-args \
       -p safety_dist:=0.45 \
       -p return_dist:=0.80 \
       -p required_safe_samples:=6 \
       -p min_ftg_hold:=0.6 \
       -p front_window:=28 \
     > /tmp/f1tenth_state_machine.log 2>&1 & \
   echo \$! > /tmp/f1tenth_state_machine.pid"

docker exec -d "${SIM_CONTAINER_ID}" bash -lc \
  "source /opt/ros/foxy/setup.bash && \
   source /sim_ws/install/local_setup.bash && \
   nohup ros2 run state_machine opponent_detector \
     --ros-args \
       -p waypoints_path:=/sim_ws/src/pure_pursuit/racelines/arc.csv \
       -p use_track_filter:=true \
       -p track_dist_thresh:=1.25 \
       -p min_persist_frames:=3 \
       -p dynamic_threshold:=0.6 \
       -p min_ego_speed:=0.8 \
       -p hold_time:=0.5 \
     > /tmp/f1tenth_opponent_detector.log 2>&1 & \
   echo \$! > /tmp/f1tenth_opponent_detector.pid"


docker exec -d "${SIM_CONTAINER_ID}" bash -lc \
  "source /opt/ros/foxy/setup.bash && \
   source /sim_ws/install/local_setup.bash && \
   nohup ros2 run pure_pursuit controller_manager_node \
    --ros-args \
      -p waypoints_path:=/sim_ws/src/pure_pursuit/racelines/arc.csv \
      -p opp_odom_topic:=/ego_racecar/opp_odom \
      -p overtake_enable:=true \
      -p use_lidar_opponent:=true \
      -p visualize_lookahead:=true \
    > /tmp/f1tenth_controller_manager.log 2>&1 & \
  echo \$! > /tmp/f1tenth_controller_manager.pid"

docker exec -d "${SIM_CONTAINER_ID}" bash -lc \
  "source /opt/ros/foxy/setup.bash && \
   source /sim_ws/install/local_setup.bash && \
   nohup ros2 run pure_pursuit controller_manager_node \
     --ros-args \
       -r __ns:=/opp_racecar \
       -p waypoints_path:=/sim_ws/src/pure_pursuit/racelines/arc.csv \
       -p odom_topic:=/opp_racecar/odom \
       -p drive_topic:=/opp_drive \
       -p scan_topic:=/opp_scan \
       -p state_topic:=/opp_state \
       -p overtake_enable:=false \
       -p velocity_percentage:=0.28 \
       -p min_lookahead:=1.4 \
       -p max_lookahead:=2.8 \
     > /tmp/f1tenth_opp_controller_manager.log 2>&1 & \
   echo \$! > /tmp/f1tenth_opp_controller_manager.pid"

echo "[6/6] Checking ROS nodes..."
sleep 2
docker exec "${SIM_CONTAINER_ID}" bash -lc \
  "source /opt/ros/foxy/setup.bash && source /sim_ws/install/local_setup.bash && timeout 8 ros2 node list"

echo ""
echo "Race track is up."
echo "Open: http://localhost:8080/vnc.html"
echo "Launch log: docker exec ${SIM_CONTAINER_ID} tail -n 80 /tmp/f1tenth_gym_launch.log"
echo "Controller log: docker exec ${SIM_CONTAINER_ID} tail -n 80 /tmp/f1tenth_controller_manager.log"
echo "Opp controller log: docker exec ${SIM_CONTAINER_ID} tail -n 80 /tmp/f1tenth_opp_controller_manager.log"
echo "State log: docker exec ${SIM_CONTAINER_ID} tail -n 80 /tmp/f1tenth_state_machine.log"
echo "Opponent detector log: docker exec ${SIM_CONTAINER_ID} tail -n 80 /tmp/f1tenth_opponent_detector.log"
