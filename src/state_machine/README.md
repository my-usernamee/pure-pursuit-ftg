# state_machine Package

This package decides whether controller should run track mode or FTG mode.

## Edit Here

- State switching logic (`GB_TRACK` / `FTGONLY`): `state_machine/state_machine.py`
- Opponent detection + temporal tracking: `state_machine/opponent_detector.py`
- Basic front obstacle detector: `state_machine/obstacle_detector.py`

## If You Want To...

- Change FTG enter/exit thresholds: edit params in `state_machine/state_machine.py`
- Change how opponent is detected/tracked: edit `state_machine/opponent_detector.py`
