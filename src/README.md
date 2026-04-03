# src Packages

Each folder below has a dedicated purpose and exact tweak files.

- `f1tenth_gym_ros/`
  - Sim bridge package.
  - Tweak map/topic/spawn here: `f1tenth_gym_ros/config/sim.yaml`
  - Tweak bridge behavior here: `f1tenth_gym_ros/f1tenth_gym_ros/gym_bridge.py`

- `pure_pursuit/`
  - Driving control package.
  - FTG logic: `pure_pursuit/pure_pursuit/ftg_logic.py`
  - Track following: `pure_pursuit/pure_pursuit/pure_pursuit_logic.py`
  - Overtake + control wiring: `pure_pursuit/pure_pursuit/controller_manager.py`

- `state_machine/`
  - Mode switching + opponent detection package.
  - State switch logic: `state_machine/state_machine/state_machine.py`
  - Opponent detect/track: `state_machine/state_machine/opponent_detector.py`

- `particle_filter/`
  - Localization package.
  - Main localization code: `particle_filter/particle_filter/particle_filter.py`

- `gap_finder/`
  - Legacy package retained for reference.
  - Main legacy node: `gap_finder/gap_finder/gap_finder_node.py`
