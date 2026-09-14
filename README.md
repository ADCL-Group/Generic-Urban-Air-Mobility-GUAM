# Generic UAM Simulation - NASA TTT-Autonomous Systems(AS): Intelligent Contingency Management (ICM)
A MATLAB/Simulink framework that models a NASA Lift + Cruise VTOL.

## Quick Start
```matlab
% From repository root:
>> main           % launches default config params & opens GUAM model
```
Simulation output is logged to `logsout{1}` and can be mapped to a structure:
```matlab
>> SimOut = logsout{1}.Values;
```

## About the Simulation

This simulation is a generic UAM simulation. It includes a generic transition aircraft model representative of a NASA Lift+Cruise vehicle configuration.

Some of the key simulation components include:

1. A simulation architecture (e.g., signal buses) that supports the most common rigid body 6-DOF frames of reference (e.g., Earth Centered Inertial, Earth Centered Earth Fixed (ECEF), North-East-Down (NED), Navigation, Velocity, Wind, Stability, and Body)
2. A simulation architecture that contains most aerospace signals/quantities of typical interest
3. A generic architecture that readily supports swapping in and out aircraft models, sensors, actuator models, control algorithms, etc.

Simulation input (fixed) parameters are provided in a large structure `SimIn`, whereas desired tunable simulation parameters are provided using the large structure: `SimPar`. The structure `SimIn`, `SimPar`, and `SimOut` therefore contain the (fixed) simulation inputs, the (variable) simulation inputs, and the simulation outputs respectively. Some basic results plotting can be performed by running the m-file: `./vehicles/Lift+Cruise/Utils/simPlots_GUAM.m`. Simulation results animation (e.g., creation of a .avi file or similar) is available by use of the script: `./utilities/Animate_SimOut.m`.

### Simulation Modifications by ADCL ERAU

This simulation was originally forked from NASA's model. Several modifications were introduced:

- **Controller Replacement:** The default baseline controller was replaced with an Incremental Nonlinear Dynamic Inversion (INDI) controller, following the architecture proposed by Lombaerts et al. and implemented in our work:
  * [Lombaerts et al., AIAA 2020-1619](https://arc.aiaa.org/doi/10.2514/6.2020-1619)
  * [ADCL ERAU Implementation, AIAA 2025-3489](https://arc.aiaa.org/doi/10.2514/6.2025-3489)
- **Remote Control & Visualization Blocks:** Simulink blocks were added to allow remote control inputs and visualization of the vehicle state.  
- **VTOL Scaling Script:** A dedicated script was created to scale the VTOL model based on Froude Scaling and Similitude principles. The method follows:
  * [Wolowicz, C.; Bowman, J.; Gilbert, W. – NASA TP-1534 (1979)](https://ntrs.nasa.gov/api/citations/19790022005/downloads/19790022005.pdf)  
- **Trim Script for Scaled VTOL:** An additional script was implemented to compute trim states for the scaled vehicle configurations. 

### Power System
The blocks within `PowerSystem` are Aeronomie blocks that are based on models and methods developed at Argonne National Laboratory for the simulation and analysis of low-carbon aircraft. For additional details on the system models, see:

* [“Urban Air Mobility: A Preliminary Case Study for Chicago and Atlanta,” IEEE, 2022.](https://ieeexplore.ieee.org/document/9814052)
* [“Estimating Mission-Based Energy and System Dynamics of e-VTOL Aircraft,” AIAA AVIATION, 2023.](https://arc.aiaa.org/doi/10.2514/6.2023-3922)
* [“Simulating System and Powerplant Dynamics of an e-VTOL Aircraft for a Full Flight Mission Including Transition,” AIAA AVIATION FORUM AND ASCEND, 2025.](https://arc.aiaa.org/doi/10.2514/6.2025-3558)
* Aeronomie: [https://vms.taps.anl.gov/tools/aeronomie/](https://vms.taps.anl.gov/tools/aeronomie/)

### Personalization
While demonstration scripts are provided, users can customize various subsystems through the `userStruct` structure and `simSetup.m` script. `userStruct` allows setting different subsystem variants and switches to tailor simulations to specific needs. Key configuration options include actuator type, atmosphere model, trajectory input, and more.

For example, a typical `userStruct` setup might look like this:
```matlab
userStruct.variants:
    refInputType: Timeseries
    vehicleType: LiftPlusCruise
    expType: DEFAULT
    atmosType: US_STD_ATMOS_76
    turbType: None
    ctrlType: BASELINE
    actType: FirstOrder
    propType: None
    fmType: Polynomial
    eomType: Simple
    sensorType: None
```
For detailed instructions on setting each option, reference input types, customizing the aero-propulsive model, and other aspects, please see the [Personalization Guide](Documentation/Personalize.md).

### Simulation Trimming
The (**offline**) trim routines are found in the `./vehicles/Lift+Cruise/Trim` folder. The top-level trim routine is: `trim_helix.m`. This script was used to trim the overactuated Lift+Cruise vehicle using the polynomial aero-propulsive database. NOTE: the routine could also be used for trimming with the strip theory S-function aero-propulsive model, but the code has not been modified to switch between the models (likely not functional using the S-function model). In the top-level `trim_helix.m` script, the user specifies a range of forward and vertical velocities (could also provide a turn radius). Next, the user provides some quantities needed for the quadratic cost function/optimization (e.g., initial guess, offset, scaling, and free variables). The quadratic cost function used by `fmincon` is: `mycost.m`, and the non-linear constraints function is `nlinCon_helix.m`. The results of the trim table schedule are then saved in a `.mat` file.

### Controller Architecture
The controller implemented in this simulation is based on the Incremental Nonlinear Dynamic Inversion (INDI) framework as described in [AIAA 2025-3489](https://arc.aiaa.org/doi/10.2514/6.2025-3489). The implementation follows the control architecture originally proposed by **Thomas Lombaerts et al.** in [AIAA 2020-1619](https://arc.aiaa.org/doi/10.2514/6.2020-1619), which was developed for unified, full-envelope flight control of eVTOL vehicles.

### Failure Configuration
Inject surface jams, engine cut-outs, sensor biases and more using the failure API described in [Failures Documentation](Documentation/Failures.md).

## Further Reading

| Area                       | Markdown                                                         |
| -------------------------- | ---------------------------------------------------------------- |
| Personalisation guide      | [Documentation/Personalize.md](Documentation/Personalize.md)     |
| Failure-injection API      | [Documentation/Failures.md](Documentation/Failures.md)           |
| Linearisation process      | [Documentation/Linearization.md](Documentation/Linearization.md) |
| S-Function build           | [Documentation/SFunction.md](Documentation/SFunction.md)         |
| Vehicle scaling            | [Documentation/Scaling.md](Documentation/Scaling.md)             |
| Trim file creation         | [Documentation/Trim.md](Documentation/Trim.md)                   |
| Frames & symbols           | [Documentation/RefFrames.md](Documentation/RefFrames.md)         |

### Related Publications
* [VTOL Urban Air Mobility Concept Vehicles for Technology Development](https://arc.aiaa.org/doi/10.2514/6.2018-3847)
  Introduces NASA's Lift+Cruise concept.
* [A Strip Theory Approach to Dynamic Modeling of eVTOL Aircraft](https://arc.aiaa.org/doi/10.2514/6.2021-1720)
  Describes the derivation of a strip theory aerodynamic model.
* [Full-Envelope Aero-Propulsive Model Identification for Lift+Cruise Aircraft Using Computational Experiments](https://arc.aiaa.org/doi/10.2514/6.2021-3170)
  Describes the derivation of a polynomial-based aerodynamic model.

