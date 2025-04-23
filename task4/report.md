# Task 4 - Decision and Planning

## Contribution Breakdown

Team Members:
- Lukas Kurz (K12007739)
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

## Behavioural Planning

The behavioural planning component uses a Finite State Machine (FSM) to manage the vehicle's high-level actions based on the environment, specifically handling static obstacles and intersections requiring a stop.

*   **States:** The FSM includes states like `LANE_FOLLOWING`, `DECELERATE_TO_STOP`, and `STOP`.
*   **Transitions:** Logic is implemented to transition between these states based on conditions such as proximity to a stop line or obstacle, and elapsed time at a stop. The required transitions are:
    *   `LANE_FOLLOWING` -> `DECELERATE_TO_STOP` (triggered when approaching a required stop)
    *   `DECELERATE_TO_STOP` -> `STOP` (triggered upon reaching the stopping point)
    *   `STOP` -> `LANE_FOLLOWING` (triggered after the required stop duration)
*   **Goal Management:** Goal points (location and speed) are defined differently for each state (e.g., nominal speed in `LANE_FOLLOWING`, zero speed at the stop line in `DECELERATE_TO_STOP` and `STOP`). Lookahead distances are used to anticipate required actions.

*(A graph visualizing the FSM states and transitions would be beneficial here).*

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

A velocity profile dictates the target speed along the chosen path.

*   **State-Dependent Profiles:** The profile generation logic adapts to the current FSM state.
    *   **`LANE_FOLLOWING`:** A nominal trajectory is calculated to adjust the vehicle's speed towards a target cruising speed.
    *   **`DECELERATE_TO_STOP`:** A deceleration trajectory is generated to bring the vehicle to a smooth stop at the designated stop line. The required deceleration is calculated based on the distance to the stop line.
    *   **`STOP`:** Velocity remains zero.
*   **Calculations:** The process involves calculating the necessary distances and target speeds at various points along the path to create smooth acceleration or deceleration profiles suitable for the current driving context (stop sign handling takes precedence over nominal lane following).

## Analysis

*(This section should contain a detailed analysis of the results observed during simulation runs. Discuss the effectiveness of the FSM logic, the path generation and selection process, the smoothness of the velocity profiles, collision avoidance performance, junction handling, and overall system behavior. Include observations on successes, failures, or areas for potential improvement based on the simulation outcomes.)*

