# RobotX Mission State Machine

ROS 2 Python package for the main planner that coordinates OCS, UAV, and USV.

The package is currently a buildable scaffold. The state-transition and topic
handling logic will be implemented in the next step.

## Build

```bash
colcon build --packages-select robotx_mission_state_machine
source install/setup.bash
```

## Run

```bash
ros2 launch robotx_mission_state_machine mission_state_machine.launch.py
```
