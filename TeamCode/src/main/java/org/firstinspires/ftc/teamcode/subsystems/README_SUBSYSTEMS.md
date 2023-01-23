# SUBSYSTEMS:

## List:

With specialized methods for each subsystem

- Belt
  - `moveNoCorrection(enum/int)` to remove drift correction algorithm
- Claw
- Lift
  - `getTarget()`
- TurnTable

## Global Methods:

Supported in all subsystems

- `getPosition()`
- `move()`
  - Supports `int` and `enum` input with type inference
- `reset()`