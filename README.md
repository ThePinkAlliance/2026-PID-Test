**2026-PID-Test is a base example for the Pink PID Arm Test Stand.  The project is based on WPI Java Command Template**

**Components**
  - REV NEO Vortex Motor (SparkFlex)
  - REV Through Bore Encoder (DIO A/B Wiring)
  - Switch, Physical (Arm Starting Position)
  - Switch, Magnetic (Arm End Position)
  - Pigeon 2.0 IMU (Arm rotates on X Axis)

**Connectivity**
  - Requires CAN Bus connectivity for:
    - Pivot motor controller
    - Pigeon 2.0 IMU
  - Requires Digital IO connectivity for:
    - 2 ports for Through Bore Encoder
    - 1 port for Start Switch
    - 1 port for End Switch
