# Task 4 - Decision and Planning

## Contribution Breakdown

Team Members:
- Lukas Kurz (K12007739) - 
- Shamekh Al-Suwi (K12146739)
- Tobias Washüttl (K11916576)
- Daniel Buchberger (K0885317)

_Note: We made use of AI assistance/LLMs for parts of the code to organize, clean up and help with documenting it for easier readability, as is standard practice nowadays. This does not in any way mean that code was plagiarized or copied, unless explicitly stated. All work was done in best conscience by the contributors named above._

## Project Overview

The goal of this project is to understand and implement key components of the hierarchical planner system found in a autonomous vehicle, consisting of a Behaviour Planner and Motion Planner.

The primary goals are to enable an autonomous vehicle to:
1. Avoid collisions with stationary objects.
2. Manage various types of junctions, including mandatory stops.
3. Maintain the centre line of the driving lane.

This is achieved through implementing a (limited) Finite State Machine for behavioural logic, a cost function evaluation for path selection, a path generation using cubic spirals with lateral offsets, and velocity profile generation that works with different driving states (lane following, deceleration to stop).

## Setup

To run this project yourself, you can view the install guidelines detailed in the file under `docs`. The files we're prepared and given to us as part of the project, so they should work as is. The basic setup is a running CARLA server, which is a simulation environment, like a video game, for driving and programmatically controlling a vehicle in a virtual environment. In our setup, this CARLA server is deployed locally using containerization based on docker and the nvidia cuda toolkit. You have to install docker, nvidia and cuda drivers for GPU support (mandatory on linux), as well as the necessary tooling to pass the GPU through to the container. This way the installation should be less of a hassle than installing CARLA natively on your machine.

_Note: If you experience issues with GPU support and the carla instance inside docker, try running the containers in privileged mode, if you have a rootless docker configuration running. Also pass the display environment variable to CARLA, instead of a offscreen method, using `-E DISPLAY=$DISPLAY`in the docker run command._

After having installed the necessary tooling, you can try to start CARLA using the `run_carla.sh` file. If the shell file won't execute, then you are probably experiencing issues with line endings in the file. You can fix that by running `sed -i 's/\r$//' run_carla.sh`. If everything works, then the scripts should keep running and depending on your setup, open up a window, where you can see the virtual environment.

Now to interact with the environment and run tests, there are approaches provided by the project template, either using a conda environment, or by running everything inside a devcontainer. Since most of us experienced issues with the conda approach, we recommend the devcontainer setup, since that worked best for us. For that you need to run VSCode, have the Devcontainer and Remote SSH extensions installed and open the `task4` folder. VSCode usually detects the devcontainer setup automatically and prompts you to reopen the folder inside a devcontainer, but if not, you can press the blue button on the bottom left, or open the command prompt and select `Reopen in Container` using the provided devcontainer config. VSCode should now spin up a new container environment for you that has all the necessary tooling and libraries installed and running. All that is left to do, is run the `SimulatorAPI.py` file using `python` to see the working result.

## Behavioral Planning

### What are Finite State Machines (FSM)?

A Finite State Machine (FSM) is a computational model used to represent and control execution flow. It consists of a finite number of states, transitions between those states, and actions. In the context of autonomous vehicles, FSMs are used to model the decision-making process for different driving scenarios.

### States of our Behavioral Planning FSM

Our FSM consists of three main states:

1. **LANE FOLLOWING**: The default state where the vehicle follows the lane at nominal speed.
2. **DECELERATION STOP**: An intermediate state where the vehicle is decelerating to come to a complete stop.
3. **STOP**: The vehicle is completely stopped for a required duration (5 seconds) before proceeding.

### Transitions Between States

The transitions between states are governed by specific conditions:

1. **LANE FOLLOWING → DECELERATION STOP**: Triggered when an object or intersection requiring a stop is detected within the lookahead distance.
2. **DECELERATION STOP → STOP**: Occurs when the vehicle has decelerated and reached the stopping point.
3. **STOP → LANE FOLLOWING**: Happens after the vehicle has remained stopped for the required 5 seconds.

### FSM Diagram

```
    ┌─────────────────┐          ┌───────────────────┐         ┌─────────────┐
    │                 │  Object  │                   │ Reached │             │
    │ LANE FOLLOWING  ├─────────►│ DECELERATION STOP ├────────►│    STOP     │
    │                 │ Detected │                   │  Stop   │             │
    └─────────┬───────┘          └───────────────────┘         └──────┬──────┘
              ▲                                                       │
              │                                                       │
              └───────────────────────────────────────────────────────┘
                            After 5-second wait complete
```

### Implementation Details

- **Lookahead Distance**: We set an appropriate lookahead distance to detect objects and intersections ahead of time.
- **Goal Setting**: For deceleration, we set a goal slightly behind the stopping point to ensure proper stopping behavior.
- **Speed Control**: We implemented different speed targets for each state (nominal for LANE FOLLOWING, zero for STOP).
- **State Transitions**: We use distance-based conditions rather than speed-based conditions for more reliable transitions.

## Path and Trajectory generation using Cubic Spirals

Since we chose a split path and velocity planning, this section deals with the path or trajectory planning of the vehicle.

Trajectories are represented as curves, specifically cubic curves. While we can represent them as splines, so X and Y as a function of time t, there is the spiral representation, which describes the curvature of the curve itself along its path. Since our representation is a polynomial of 3rd order, the name for the representation is **Cubic spiral**.

In our case, we use them to generate smooth paths between a starting pose (position, orientation, curvature) and an ending pose. Their attributes help us with a smooth continuous curvature, which prevents sudden movements or strong jerks, which can become uncomfortable or worrying for people riding the vehicle.

While you would ideally drive on the centerline of your lane at a certain lookahead distance, this is 
not always possible or practical, due to cars or obstacles, which is why we pursue mulitple, laterally 
offset paths or goals, that generated on each side of the ideal goal. For this, we first calculate the ideal goal, which is on the center of our lane at a certain lookahead distance. Then we create multiple alternative goals, each offset at a predefined offset distance laterally from the ideal goal, and then this produces multiple candidates for our next path.

One of the most vital parts of the system is the collision checking, since it important for the safety of car and driver. First our trajectory is split into multiple discrete piece of appropriate length, not to small to be computable, but fine and detailled enough to not leave gaps. These discrete steps are then checked for collisions.

For collision checking we use a circle based approach, where we try to fit a cars shape into one or a series of circles. This way we can firstly quickly check using centerpoint distances of the circles, and when these distances are to close, we check for actual intersection of circles i.e. vehicles. We also do static and dynamic collision checking, so for static objects that do not move such as trees and more difficult checking on dynamic objects, such as cars, where we have to create predictions for their trajectories to compare against our own for collision.

Lastly, to calculate the trajectories, we use a cost function to evaluate them and choose the best trajectory. This cost function increases, i.e. penalizes dangerous paths that might lead to collisions by coming to closes to obstacles, so the distance to them is used a proxy measure for that. The cost reduces and therefore rewars paths that are efficient and stay close to the center line.

## Velocity Profile generation

### 1. **Brief Explanation of Velocity Profile Generation**

The **Velocity Profile Generator** computes a sequence of velocities (a velocity profile) for a vehicle along a given path (the "spiral") to execute various driving maneuvers safely and comfortably.  
The goal is to transition from the current speed to a target speed—either by accelerating, decelerating, or coming to a stop—depending on the driving scenario.

Depending on the maneuver, different profiles are generated:
- **Nominal/Lane Follow:** The vehicle accelerates or decelerates to a desired speed.
- **Decelerate to Stop:** The vehicle smoothly decelerates to a stop.
- **Follow Vehicle:** (Not yet implemented) The vehicle matches the speed of a lead vehicle.

### 2. **How Are the Profiles Calculated?**

a) **General Workflow**

1. **Maneuver Detection:** The generator selects the appropriate profile function based on the current driving state (e.g., stopping, following, lane keeping).
2. **Velocity Calculation:**  
   For each point along the trajectory, the target velocity is computed, based on the start speed, target speed, maximum allowed acceleration/deceleration, and the distance traveled.
3. **Time Calculation:** For each point, the relative time is also determined, resulting in a time-based trajectory.


b) **Key Calculation Formulas**

- **Distance with Constant Acceleration:**  
  \[ d = \frac{v_f^2 - v_i^2}{2a} \]  
  (Distance \(d\) to go from initial velocity \(v_i\) to final velocity \(v_f\) at constant acceleration \(a\))

- **Final Speed after a Distance:**  
  \[ v_f = \sqrt{v_i^2 + 2ad} \]  
  (Final speed \(v_f\) after distance \(d\) from initial speed \(v_i\) at acceleration \(a\))


### 3. **How Are Distances or Velocities Calculated for Each Point?**

**a) Distance Calculation Between Points**

- The distance between two consecutive points on the trajectory is computed using the helper function `path_point_distance()`.
- For many calculations (such as when to start braking), the cumulative distance along the trajectory is summed.

**b) Velocity Calculation for Each Point**

- **Acceleration/Deceleration Phase:**  
  For each segment, the new velocity is calculated using the above formula (\(v_f\)), based on the distance to the next point and the current velocity.
- **Constant Speed Phase:**  
  Once the target speed is reached, the velocity remains constant for the rest of the trajectory (unless further deceleration is needed).

**c) Time Calculation**

- The time between two points is calculated as:  
  \[ \Delta t = \frac{|v_{i+1} - v_i|}{a_\text{max}} \]  
  (during acceleration/deceleration)  
  or  
  \[ \Delta t = \frac{\text{distance}}{v} \]  
  (during constant speed)


### 4. **Typical Workflow for Each Maneuver**

**a) Decelerate to Stop**
- Calculate how much distance is needed to first decelerate to a "slow speed," then to a full stop.
- If the required braking distance exceeds the remaining path, the deceleration is adjusted so that the vehicle stops exactly at the end of the path.
- Otherwise, the profile consists of three phases: deceleration to slow speed, constant slow speed, and final braking to zero.

**b) Nominal Trajectory (Lane Follow)**
- Calculate the distance required to transition from the current speed to the target speed.
- Up to this point, the vehicle accelerates or decelerates; after that, it maintains the target speed.


### 5. **Example Workflow (Pseudocode)**

For each maneuver:
1. Determine start and target speed.
2. Calculate required distance for acceleration/deceleration phase.
3. Iterate over the trajectory, for each point:
    - Calculate distance to the next point.
    - Compute new velocity using the kinematic formula.
    - Compute time to the next point.
    - Add all values as a TrajectoryPoint to the trajectory.


### 6. **Key Methods in the Code**

- **`calc_distance(v_i, v_f, a)`**: Calculates the distance needed to go from \(v_i\) to \(v_f\) at acceleration \(a\).
- **`calc_final_speed(v_i, a, d)`**: Calculates the final speed after distance \(d\) from \(v_i\) at acceleration \(a\).
- **`decelerate_trajectory()`**: Generates a profile for smooth stopping.
- **`nominal_trajectory()`**: Generates a profile for reaching a target speed.
- **`generate_trajectory()`**: Selects the appropriate profile function based on the maneuver.


### 7. **Summary**

The Velocity Profile Generator creates a physically plausible velocity profile for each driving maneuver by applying basic kinematic equations and respecting comfort/safety constraints (e.g., max acceleration).  
The calculation is performed point-by-point along the planned route, providing for each point both the velocity and the corresponding time.


## Analysis

### System Integration and Performance

Our integrated planning system successfully handles various driving scenarios, including lane following, stopping at intersections, and navigating around obstacles. The three-layer approach (behavioral planning, path generation, and velocity profiling) provides a robust and flexible framework for autonomous navigation.

### Strengths of the Implementation

1. **Robust State Machine**: The FSM design clearly separates different driving behaviors, making the system more maintainable and easier to debug. The state transitions are well-defined and based on reliable distance metrics rather than potentially noisy speed measurements.

2. **Adaptable Path Planning**: The cubic spiral approach generates smooth paths that respect vehicle kinematic constraints. Multiple candidate paths with different lateral offsets provide flexibility in obstacle avoidance while maintaining comfort.

3. **Safety-First Design**: Comprehensive collision checking ensures that only safe paths are selected. The weighted cost function balances multiple objectives, prioritizing safety while considering efficiency and comfort.

4. **Smooth Velocity Profiles**: The two-phase velocity profile generation creates comfortable acceleration and deceleration patterns. State-specific velocity calculations ensure appropriate speed control in different scenarios.

### Challenges and Limitations

1. **Computational Complexity**: Generating and evaluating multiple spiral paths can be computationally intensive, potentially limiting real-time performance on less powerful hardware.

2. **Parameter Tuning**: The system requires careful tuning of parameters like lookahead distance, maximum lateral offsets, and cost weights. Finding the optimal balance between different objectives requires extensive testing.

3. **Edge Cases**: While the system handles common scenarios well, unusual edge cases (like extremely sharp turns or complex intersection geometries) might require additional handling.

4. **Reactivity vs. Planning Horizon**: There's an inherent trade-off between quick reaction to changing conditions and maintaining a stable plan over a longer horizon.

### Future Improvements

1. **Dynamic Parameter Adjustment**: Implementing adaptive parameter tuning based on driving context could improve performance across different environments.

2. **Prediction Integration**: Incorporating better prediction of other road users' behaviors would enhance planning in dynamic environments.

3. **Machine Learning Optimization**: Using learning-based approaches could help optimize cost functions and parameter selection based on real-world performance data.

4. **Extended State Machine**: Adding more specialized states for complex maneuvers like lane changes, overtaking, or unprotected turns would increase the system's capabilities.

### Conclusion

Our implementation successfully addresses the core requirements of behavioral planning, path generation, and velocity control for autonomous navigation. The modular design allows for incremental improvements and extensions as needed. Through careful integration of these components, we've created a planning system that balances safety, comfort, and efficiency in a wide range of driving scenarios.