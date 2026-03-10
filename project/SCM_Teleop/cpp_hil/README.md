# C++ Hardware-in-the-Loop Demo

C++ implementation of the SCM teleoperation demo using Chrono HIL.

## Files

| File | Description |
|------|-------------|
| `proj_HIL_scm_teleop.cpp` | Main C++ demo with SCM terrain and sensor visualization |
| `Ros2Bridge.cpp/h` | Optional ROS2 bridge for external integration |

## Building

From the chrono-HIL build directory:

```bash
cd build
cmake ..
make proj_HIL_scm_teleop
```

## Running

```bash
./bin/proj_HIL_scm_teleop
```

## Features

- Real-time deformable terrain simulation
- Sensor visualization (cameras, lidar)
- SDL joystick/steering wheel input
- Optional ROS2 telemetry bridge

## Dependencies

- Chrono (Vehicle, Sensor, Irrlicht, SynChrono)
- SDL2
- ROS2 (optional, for Ros2Bridge)
