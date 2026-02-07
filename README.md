2026-PID-Test is a base example for testing/learning on the PID Arm Test Stand.
PID Test Stand
- Components
  - REV NEO Vortex Motor (SparkFlex)
  - REV Through Bore Encoder (DIO A/B Wiring)
  - Switch, Physical (Arm Starting Position)
  - Switch, Magnetic (Arm End Position)
  - Pigeon 2.0 IMU (Arm rotates on X Axis)
- Robot Agnostic: can run on Rio 1 or Rio 2 robot.
  - Requires CAN Bus connectivity for:
    - Pivot motor controller
    - Pigeon 2.0 IMU
- Project is based on WPI Java Command Template
