# HaldexController

PC simulator for the Haldex Gen5 AWD torque controller.

The goal is to run the production controller code on a PC, feed it real CAN bus data recorded during vehicle testing, tune the calibration parameters, and finally compile a static library ready to be deployed to the real vehicle ECU.

---

## How it works

```
[Real vehicle]
     |
     | CAN logger (OBD / custom hardware)
     v
 [.csv / .log file]
     |
     v
[HaldexController PC simulator]
     |
     | compare output vs. real behavior
     | adjust HaldexControlConfig parameters
     v
[Tuned calibration]
     |
     v
[Static library  →  Vehicle ECU]
```

1. **Record** — drive the vehicle and log CAN bus data (wheel speeds, yaw rate, throttle, steering, etc.)
2. **Simulate** — replay the log through the simulator and inspect the lock % output per frame
3. **Tune** — adjust `HaldexControlConfig` parameters for NORMAL and SPORT modes
4. **Deploy** — build as a static library and flash to the ECU

---

## Project structure

```
src/
  core/         — main controller logic (signal processing, state estimation, lock calculation, safety)
  types/        — CAN input, config, physics, and state structs
  logging/      — diagnostics and telemetry output
  main.cpp      — PC entry point (replay loop)
tests/
  unit/         — isolated tests for each calculation stage
  simulation/   — full-loop tests with synthetic sensor scenarios
scripts/        — build.bat (Windows) / build.sh (Mac)
CMakeLists.txt
BUILDING.md     — full build instructions
```

---

## Building

See [BUILDING.md](BUILDING.md) for prerequisites and step-by-step instructions for Windows and Mac.

Quick start:

```bat
# Windows
scripts\build.bat

# Mac / Linux
scripts/build.sh
```

---

## Input data format

The simulator reads a frame-by-frame CAN snapshot. Each frame maps to `CanInputLayer`:

| Field                    | Unit       | Description                          |
|--------------------------|------------|--------------------------------------|
| `vehicleSpeedKmh`        | km/h       | Vehicle reference speed              |
| `wheelSpeedFl/Fr/Rl/Rr`  | km/h       | Individual wheel speeds              |
| `throttlePct`            | 0–100 %    | Accelerator pedal position           |
| `steeringAngleDeg`       | degrees    | Steering column angle                |
| `gear` / `reverseGear`   | —          | Current gear and reverse flag        |
| `actualTorqueSumNm`      | Nm         | Engine torque delivered              |
| `yawRateDegS`            | °/s        | Yaw rate from IMU                    |
| `lateralAccelG`          | g          | Lateral acceleration from IMU        |
| `longitudinalAccelG`     | g          | Longitudinal acceleration from IMU  |
| `brakePressure`          | bar        | Brake circuit pressure               |
| `absActive`              | bool       | ABS intervention flag                |
| `escOff`                 | bool       | ESC-off mode active                  |
| `selectedDriveMode`      | 0/1        | NORMAL = 0, SPORT = 1                |

---

## Configuration

All tunable parameters live in `HaldexControlConfig` (`src/types/configTypes.hpp`). A separate config exists for each drive mode:

```cpp
HaldexCalibration calib;
calib.configs[NORMAL] = { ... };
calib.configs[SPORT]  = { ... };
setupHaldexControl({ calib.configs[NORMAL], calib.configs[SPORT] });
```

Key parameter groups:

| Group                  | Examples                                                  |
|------------------------|-----------------------------------------------------------|
| Base lock              | `baseLockLowSpeed`, `baseLockHighSpeedCutMps`             |
| Proactive / throttle   | `throttleProactiveGain`, `maxFeedforwardLockCap`          |
| Reactive slip          | `slipReactiveGain`, `slipTriggerThresholdMps`             |
| Lateral dynamics       | `chassisBalanceGain`, `balanceUndersteerThresholdRadS`    |
| Slew rate              | `maxLockSlewRateUp`, `maxLockSlewRateDown`                |
| ABS behaviour          | `absMaxFloorRatio`, `absFadeTauS`                         |
| Parking / low speed    | `steeringReductionMaxAngleRad`, `parkingDeactivationAngleRad` |
| Signal filtering       | `signalFilterTimeConstant`, `steeringRateFilterTime`      |

---

## Running tests

```bat
# Windows (from project root)
cd build
ctest --output-on-failure -C Release -V

# Mac / Linux
cd build
ctest --output-on-failure -V
```

---

## Roadmap

- [ ] CSV replay loader with configurable column mapping
- [ ] Telemetry export (lock %, slip, yaw error) for plotting
- [ ] Static library build target for ECU deployment
- [ ] Additional drive modes (e.g. SNOW, TRACK)
- [ ] CI with real-world log regression tests
