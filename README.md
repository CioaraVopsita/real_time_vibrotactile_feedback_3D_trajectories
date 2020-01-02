# Real-time vibrotactile feedback for 3D trajectories

Vibrotactile feedback is given independently for two movement components, position and velocity, by comparing the real-time values to a target path and target velocity profile respectively. Compound paths are created by introducing via-points along the trajectory, with the target path being dynamically constructed for each individual to account for variables such as height, arm length, etc. The target velocity profile is constructed using the minimum-jerk model.<sup>1</sup> Velocity is used as a proxy for quantifying movement smoothness which represents a measure of movement optimality.

The code also contains a movement decomposition algorithm for 3D trajectories.<sup>2</sup> Any trajectory can be regarded as a linear combination of minimum-jerk submovements. The submovements are determined by minimising the error between the parameter-dependent reconstructed tangential velocity profile and the actual tangential velocity profile of the movement.

## Prerequisites

- Polhemus Liberty motion tracker 
- Matlab R2007b 
- Haptic actuators

## Collecting data

The steps for running the algorithms are detailed below:
- For **both** the experimental and the control condition:
  - Set the desired number of via-points points in **starting_point.m** by specifying the fraction points along the trajectory at which the via-points should occur (m1, m2, etc).
  - Set the angles in radians (theta1, theta2, etc) at which via-points should be introduced in **starting_point.m** (rotation around z-axis).
  - Run **stationary_collection.m** to record the starting point of a participant.
  - Run **starting_point.m** to compute the coordinates of the via-points.
  
### Experimental condition
- In **concurrent_feedback.m** replace the coordinates of the via-points in the *intersection_point* variable.
- Run **concurrent_feedback.m**.

### Control condition
- In **offline_feedback.m** replace the coordinates of the via-points in the *intersection_point* variable.
- Run **offline_feedback.m**.

### Movement decomposition
- Run **decompose_movement.m** that will search for the minimum of the *submovement* function in **submovement.m**.
- In turn, **submovement.m** constructs the submovements using the parameters to be optimised based on the mathematical equations in **minjerk.m**.
- For this algorithm, several parameters should be specified in **decompose_movement.m**: number of submovements, time vector, raw movement x, y, z velocity components.
``` Matlab
> no_submovements = estimated_number_of_submovements;
> time = time_data
> Gx = recorded_velocity_x_coordinate
> Gy = recorded_velocity_y_coordinate
> Gz = recorded_velocity_z_coordinate
```
- **!!** All vectors specified above should be row vectors.

## Specifics and limitations

- Bursts of 0.1 seconds of vibrotactile feedback are given to participants for correct position and velocity. 
- The system works for **_any_** number of via-points without the need for further modifications.
- At the moment, the system only processes data from __one__ motion tracker. Minor changes are required the raw data processing section (KF, sdKF, Gaussian smoothing, peak detection) to scale the system to multiple motion trackers.
- For multiple motion trackers, *Parallel Processing* MATLAB toolbox is recommended to avoid delays in data processing.

## Authors
- Nicoleta Condruz - feedback and movement decomposition algorithms
- Prof. Chris Miall - setting up the Liberty motion tracker and the Mini-Link Distribution Box for the linear resonance actuators

## Acknowledgments 
Very grateful to Prof. Chris Miall who has always made time to discuss and debug the project and remained confident that I would make it work.

## Sources
[1]T. Flash and N. Hogan, “The coordination of arm movements: an experimentally confirmed mathematical model,” The Journal of Neuroscience, vol. 5, no. 7, pp. 1688–1703, Jul. 1985.

[2]J. Y. Liao and R. F. Kirsch, “Characterizing and Predicting Submovements during Human Three-Dimensional Arm Reaches,” PLoS ONE, vol. 9, no. 7, p. e103387, Jul. 2014.
 
