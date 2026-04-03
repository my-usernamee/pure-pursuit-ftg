# pure_pursuit Package

This package drives the ego car in track mode and handles overtake logic.

## Edit Here

- Main controller flow (track mode, FTG mode handoff, overtake): `pure_pursuit/controller_manager.py`
- FTG behavior: `pure_pursuit/ftg_logic.py`
- Raceline target selection + steering output: `pure_pursuit/pure_pursuit_logic.py`
- Raceline CSVs: `racelines/`

## If You Want To...

- Tune FTG safety/aggressiveness: edit `pure_pursuit/ftg_logic.py`
- Tune overtake distance/side offset: edit params in `pure_pursuit/controller_manager.py`
- Tune normal track following: edit `pure_pursuit/pure_pursuit_logic.py`
