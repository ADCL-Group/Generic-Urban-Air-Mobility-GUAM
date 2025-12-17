# Autonomous flight setup and execution

This document describes how to configure and run **autonomous flight simulations** with the GUAM simulation.
Before proceeding, it is recommended to become familiar with the standard simulation workflow and [main.m](../main.m)

## Running an Autonomous Flight Simulation

### Requirements
Autonomous flight requires one of the following:
- a path planner, or
- a pre-generated trajectory / set of waypoints.

For the [autonomous example](../mainAutonomous.m), we use the **FMT\*** path planner. 
> [!TIP]
> Any trajectory or set of waypoints provided in the **NED reference frame** can be used.

### Configuring Autonomous Control
To enable autonomous flight, the aircraft must be controlled by the autopilot / autonomous flight module instead of a pilot-in-the-loop controller.
> [!WARNING]
> At the moment, autonomous flight is supported for **forward flight**.

This is configured through the simulation variants structure:
```matlab
userStruct.variants.ctrlMode = CtrlModeEnum.Autonomous;
```
This switches the controller logic and enables the autonomous control laws in the Simulink model.

### Providing Trajectory Inputs (Workspace Variables)
The autonomous flight subsystem expects the planned trajectory to be setup via MATLAB workspace variables.

Required variables:

* `flightTrajectory`: an **n×3** array of waypoint positions along the trajectory in:

  - expected in **ENU frame**
  - expected in **meters**
  - columns: `[East, North, Up]`

* `trajLen`: scalar equal to `n` (number of points in `flightTrajectory`)

* `resetFlag`: scalar flag used to trigger trajectory interpolation inside the model.

  - Set `resetFlag = 0`
  - No additional setup is required: interpolation is performed automatically at sim start by default.
  
* `SimIn.PathPlan.AirSpeed`: scalar airspeed along the trajectory. 
  - expected in **m/s**
  - It is used by the Virtual Leader block to interpolate the trajectory in time
  
Example:

```matlab
SimIn.PathPlan.AirSpeed = 50; % [m/s]
flightTrajectory = smoothedTraj(:,1:3);   % [East North Up] in meters
trajLen          = size(flightTrajectory,1);
resetFlag        = 0;
```

### Frames and Units

The **trajectory input** for autonomous flight is provided in **ENU (meters)**, but the vehicle controller/state feedback operates internally in **NED (feet)**.

This difference is handled internally by the Simulink model in the block:

`GUAM/Vehicle Simulation/ADCL Vehicle Control/Variant Subsystem/AutoFlight/Virtual Leader`

The **Virtual Leader** block:

* reads `flightTrajectory` (ENU, meters),
* reads `SimIn.PathPlan.AirSpeed`,
* interpolates the trajectory assuming that airspeed, and
* outputs reference position and velocity at each time step **already converted** to the controller’s expected frame/units (**NED, feet**).

> [!NOTE]
> Because of this internal handling, the user only needs to provide `flightTrajectory` in ENU+m as described above.

### Setting Initial Conditions

Initial conditions are configured using the same structure as the [standard simulation](../main.m) and the provided [autonomous example](../mainAutonomous.m).
To ensure a smooth autonomous start, the simulation initial conditions are aligned with the first point of the planned trajectory:

* Position and velocity are extracted from the trajectory
* Units are converted from meters to feet
* Axis ordering is converted from ENU to the simulation’s NED reference frame
* Initial heading (`chi_des`) is set consistently with the initial heading of the trajectory

## Autonomous Flight Example
### Trajectory Generation
As mentioned above, autonomous flight requires a trajectory defined in the NED reference frame.
In [mainAutonomous.m](../mainAutonomous.m), a path is generated using the FMT* planner and then used as the reference for the controller. This is done by:

1. Building an occupancy map from OSM data (an example map is provided in the FMT repository)
2. Running an FMT* planner to compute a feasible path
3. Smoothing the resulting trajectory
4. Computing heading along the path

The resulting trajectory is stored in:

```matlab
SimIn.PathPlan
```

and later converted into the reference format expected by the simulation.

### FMT* path planner
The example uses the FMT* path planner, hosted under the same organization as this repository. This path planner can be obtained in one of the following ways:

- **As a git submodule** (recommended):
  ```bash
  git submodule add <FMT_PLANNER_REPO_URL> fmt_planner
  git submodule update --init --recursive
  ```

- **As a local copy**:
  Clone or download the planner repository and place it in the repo root as:

  ```
  fmt_planner/
    ├─ src/
    ├─ examples/
    └─ data/
  ```

Once you have the path planner source code, the autonomous script assumes the planner source is available under:

```
fmt_planner/src
```
and adds it to the MATLAB path using:

```matlab
addpath(genpath('fmt_planner/src'))
```
