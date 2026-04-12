# local_planner Package

Entry point:
- `local_planner/spliner.py`

Inputs:
- `/ego_racecar/odom`
- `/scan`
- `/opponent_detected`, `/opponent_detection`
- `/obstacle_detected`, `/obstacle_distance`

Outputs:
- `/planner/local_path` (`nav_msgs/Path`)
- `/planner/overtake_feasible` (`std_msgs/Bool`)
- `/planner/path_active` (`std_msgs/Bool`)
