# Mars-EDL-Autonomous-Navigation
MATLAB simulation of autonomous position estimation during Mars Entry, Descent and Landing using baseline and enhanced Extended Kalman Filters.

# Mars EDL Autonomous Navigation Simulation

A MATLAB-based academic project focused on autonomous position estimation during Mars Entry, Descent and Landing (EDL).

The project investigates the performance of Extended Kalman Filter (EKF) approaches for navigation estimation under simulated conditions, comparing a baseline implementation with an enhanced approach incorporating accelerometer bias estimation and confidence-dependent Terrain Relative Navigation (TRN) measurement weighting.

## Project Overview

The simulation explores how sensor information and measurement confidence can influence navigation estimation accuracy during a simplified Mars EDL scenario.

Two estimation approaches are evaluated:

- **Baseline EKF:** Reference navigation estimation approach.
- **Enhanced EKF:** Incorporates accelerometer bias estimation and confidence-dependent TRN measurement weighting.

A Monte Carlo simulation is used to evaluate estimation performance across multiple trials.

## Key Features

- Simplified Mars Entry, Descent and Landing trajectory simulation.
- Extended Kalman Filter-based position estimation.
- Accelerometer bias estimation.
- Simulated Terrain Relative Navigation (TRN) measurements.
- Confidence-dependent measurement weighting.
- Monte Carlo performance evaluation.
- Statistical comparison of estimation errors.

## Simulation Results

The Monte Carlo evaluation produced the following mean final position estimation errors:

| Estimation Approach | Mean Final Position Error |
|---|---:|
| Baseline EKF | 30.332 m |
| Enhanced EKF | 10.598 m |

The enhanced approach achieved an approximately **65.1% reduction** in mean final position estimation error compared with the baseline implementation.

## Technologies and Methods

- MATLAB
- Extended Kalman Filtering (EKF)
- State Estimation
- Inertial Measurement Unit (IMU) Simulation
- Terrain Relative Navigation (TRN)
- Monte Carlo Simulation
- Statistical Performance Analysis


## Project Scope and Limitations

This project is an academic simulation developed to investigate navigation estimation techniques for a simplified Mars EDL scenario.

The TRN measurements are simulated, and the clock model is evaluated separately rather than integrated into the EKF. The simulation does not represent a flight-qualified navigation system or a complete six-degree-of-freedom Mars landing model.

## Author

**Keila Fortes**

Aerospace Engineering | Beijing Institute of Technology

## Disclaimer

Developed for academic and educational purposes. Simulation results are specific to the implemented models, assumptions and test conditions.
