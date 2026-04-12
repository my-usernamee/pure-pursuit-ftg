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

echo "[2/6] Building ROS packages in sim container..."
docker exec "${SIM_CONTAINER_ID}" bash -lc \
  "source /opt/ros/foxy/setup.bash && \
   cd /sim_ws && \
   colcon build --symlink-install \
     --packages-select f1tenth_gym_ros pure_pursuit state_machine local_planner"

echo "[3/6] Stopping previous driving processes..."
docker exec "${SIM_CONTAINER_ID}" bash -lc \
  "for PID_FILE in \
      /tmp/f1tenth_controller_manager.pid \
      /tmp/f1tenth_state_machine.pid \
      /tmp/f1tenth_obstacle_detector.pid \
      /tmp/f1tenth_local_planner.pid \
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

sleep 2
echo "[5/6] Starting detection + planning stack..."

docker exec -d "${SIM_CONTAINER_ID}" bash -lc \
  "source /opt/ros/foxy/setup.bash && \
   source /sim_ws/install/local_setup.bash && \
   nohup ros2 run state_machine obstacle_detector \
     --ros-args \
       -p waypoints_path:=/sim_ws/src/pure_pursuit/racelines/korea_mintime_sparse.csv \
       -p use_track_filter:=true \
       -p track_dist_thresh:=0.35 \
       -p detect_dist:=4.50 \
       -p min_cluster_points:=1 \
       -p max_cluster_points:=60 \
       -p size_min_x:=0.02 \
       -p size_min_y:=0.02 \
       -p size_max_x:=0.60 \
       -p size_max_y:=0.60 \
       -p min_persist_frames:=1 \
       -p hold_time:=0.75 \
     > /tmp/f1tenth_obstacle_detector.log 2>&1 & \
   echo \$! > /tmp/f1tenth_obstacle_detector.pid"

docker exec -d "${SIM_CONTAINER_ID}" bash -lc \
  "source /opt/ros/foxy/setup.bash && \
   source /sim_ws/install/local_setup.bash && \
   nohup ros2 run local_planner spliner \
     --ros-args \
       -p waypoints_path:=/sim_ws/src/pure_pursuit/racelines/korea_mintime_sparse.csv \
       -p static_trigger_distance:=3.20 \
       -p opponent_trigger_distance:=0.0 \
       -p planner_lookahead_horizon:=12.0 \
       -p min_side_clearance:=1.00 \
       -p lane_half_width:=1.20 \
       -p boundary_margin:=0.38 \
       -p apex_lateral_margin:=0.45 \
       -p obstacle_half_width:=0.18 \
       -p overtake_lateral_buffer:=0.22 \
       -p static_line_d_threshold:=0.30 \
       -p static_obs_alpha:=0.20 \
       -p pre_apex_points:=[3.0,4.2,5.4] \
       -p post_apex_points:=[5.0,6.5,8.0] \
       -p path_hold_sec:=1.60 \
       -p side_lock_release_sec:=1.60 \
       -p lateral_smoothing_window:=9 \
       -p prefer_right_overtake:=true \
     > /tmp/f1tenth_local_planner.log 2>&1 & \
   echo \$! > /tmp/f1tenth_local_planner.pid"

echo "[6/6] Starting state machine + controllers..."
docker exec -d "${SIM_CONTAINER_ID}" bash -lc \
  "source /opt/ros/foxy/setup.bash && \
   source /sim_ws/install/local_setup.bash && \
   nohup ros2 run state_machine state_machine \
     --ros-args \
       -p static_trigger_distance:=3.20 \
       -p opponent_trigger_distance:=0.0 \
       -p hard_safety_distance:=0.10 \
       -p front_window:=24 \
       -p clear_distance:=1.50 \
       -p hazard_confirm_cycles:=2 \
       -p overtake_timeout_sec:=8.0 \
       -p overtake_min_commit_sec:=1.50 \
       -p planner_hold_sec:=3.00 \
       -p required_clear_cycles:=10 \
       -p overtake_lost_cycles_to_fallback:=12 \
       -p emergency_persist_cycles:=3 \
       -p ignore_opponent:=true \
     > /tmp/f1tenth_state_machine.log 2>&1 & \
   echo \$! > /tmp/f1tenth_state_machine.pid"

docker exec -d "${SIM_CONTAINER_ID}" bash -lc \
  "source /opt/ros/foxy/setup.bash && \
   source /sim_ws/install/local_setup.bash && \
   nohup ros2 run pure_pursuit controller_manager_node \
     --ros-args \
       -p waypoints_path:=/sim_ws/src/pure_pursuit/racelines/korea_mintime_sparse.csv \
       -p visualize_lookahead:=true \
       -p velocity_percentage:=0.62 \
       -p max_speed_cap:=3.60 \
       -p overtake_speed_boost:=0.10 \
       -p overtake_speed_cap:=2.10 \
       -p overtake_lookahead_scale:=0.90 \
       -p overtake_steering_limit_deg:=18.0 \
       -p max_steering_rate:=0.20 \
       -p overtake_max_steering_rate:=0.26 \
       -p gb_min_speed:=1.70 \
       -p overtake_min_speed:=1.55 \
       -p dynamic_speed_min_scale:=0.70 \
       -p dynamic_speed_steer_penalty:=0.34 \
       -p offtrack_d_limit:=0.45 \
       -p recovery_speed:=0.95 \
       -p use_ground_truth_opponent:=false \
     > /tmp/f1tenth_controller_manager.log 2>&1 & \
   echo \$! > /tmp/f1tenth_controller_manager.pid"

echo "Checking ROS nodes..."
sleep 2
docker exec "${SIM_CONTAINER_ID}" bash -lc \
  "source /opt/ros/foxy/setup.bash && source /sim_ws/install/local_setup.bash && timeout 8 ros2 node list"

echo ""
echo "Race stack is up."
echo "Open: http://localhost:8080/vnc.html"
echo "Launch log: docker exec ${SIM_CONTAINER_ID} tail -n 80 /tmp/f1tenth_gym_launch.log"
echo "State log: docker exec ${SIM_CONTAINER_ID} tail -n 80 /tmp/f1tenth_state_machine.log"
echo "Planner log: docker exec ${SIM_CONTAINER_ID} tail -n 80 /tmp/f1tenth_local_planner.log"
echo "Controller log: docker exec ${SIM_CONTAINER_ID} tail -n 80 /tmp/f1tenth_controller_manager.log"
