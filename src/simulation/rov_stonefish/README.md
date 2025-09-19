# rov_stonefish

## Description

Purpose of this package is to create simulation environment for various scenarios

## Launch

This package has one `*.launch.py` file. 

```bash
windturbine_bluerov2.launch.py
wreckage_bluerov2.launch.py
```

### windturbine_bluerov2.launch.py

- launches `bluerov2` robot description
- subcribes to `ds4` controller
- launches `rov_thruster_manager` node
- launches `windturbine` scene with `bluerov2` robot.
- opens `rvi2` with `tank_bluerov2_imu.rviz` configuration

#### Parameters

- `config` (default: *windturbine_bluerov2.yaml*) - path to config file
- `rviz_config` (default: *tank_bluerov2_imu.rviz*) - path to rviz config file

### wreckage_bluerov2.launch.py

Map of wreckage prepared for exhibition

![Map](./docs/images/Screenshot%20from%202025-04-01%2010-11-50.png)

![rviz](./docs/images/Screenshot%20from%202025-04-01%2010-15-25.png)


- launches `bluerov2` robot description
- subcribes to `ds4` controller
- launches `rov_thruster_manager` node
- launches `windturbine` scene with `bluerov2` robot.
- opens `rvi2` with `tank_bluerov2_imu.rviz` configuration

#### Parameters

- `config` (default: *wreckage_bluerov2.yaml*) - path to config file
- `rviz_config` (default: *tank_bluerov2_imu.rviz*) - path to rviz config file
