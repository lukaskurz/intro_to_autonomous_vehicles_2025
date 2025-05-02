# Task 5 - Control

## Contribution Breakdown

Team Members:
- Lukas Kurz (K12007739) - Setup, PID Tuning, Report
- Tobias Washüttl (K11916576) - PID Controller
- Daniel Buchberger (K0885317) - Throttle and Steering Control

_Note: We made use of AI assistance/LLMs for parts of the code to organize, clean up and help with documenting it for easier readability, as is standard practice nowadays. This does not in any way mean that code was plagiarized or copied, unless explicitly stated. All work was done in best conscience by the contributors named above._

## Setup

_Same as in Task 4_

To run this project yourself, you can view the install guidelines detailed in the file under `docs`. The files we're prepared and given to us as part of the project, so they should work as is. The basic setup is a running CARLA server, which is a simulation environment, like a video game, for driving and programmatically controlling a vehicle in a virtual environment. In our setup, this CARLA server is deployed locally using containerization based on docker and the nvidia cuda toolkit. You have to install docker, nvidia and cuda drivers for GPU support (mandatory on linux), as well as the necessary tooling to pass the GPU through to the container. This way the installation should be less of a hassle than installing CARLA natively on your machine.

_Note: If you experience issues with GPU support and the carla instance inside docker, try running the containers in privileged mode, if you have a rootless docker configuration running. Also pass the display environment variable to CARLA, instead of a offscreen method, using `-E DISPLAY=$DISPLAY`in the docker run command._

After having installed the necessary tooling, you can try to start CARLA using the `run_carla.sh` file. If the shell file won't execute, then you are probably experiencing issues with line endings in the file. You can fix that by running `sed -i 's/\r$//' run_carla.sh`. If everything works, then the scripts should keep running and depending on your setup, open up a window, where you can see the virtual environment.

Now to interact with the environment and run tests, there are approaches provided by the project template, either using a conda environment, or by running everything inside a devcontainer. Since most of us experienced issues with the conda approach, we recommend the devcontainer setup, since that worked best for us. For that you need to run VSCode, have the Devcontainer and Remote SSH extensions installed and open the `task4` folder. VSCode usually detects the devcontainer setup automatically and prompts you to reopen the folder inside a devcontainer, but if not, you can press the blue button on the bottom left, or open the command prompt and select `Reopen in Container` using the provided devcontainer config. VSCode should now spin up a new container environment for you that has all the necessary tooling and libraries installed and running. All that is left to do, is run the `SimulatorAPI.py` file using `python` to see the working result.

## Control

### What is Control

Control refers to how physical controls of the car are actuated based on the commands from the planning system. It bridges the perception and planning systems to the actual physical vehicle movement. Control system work by being given a desired state, like steering angle or velocity, comparing them to the actual physical state of the vehicle, calculating an error and then try to correct that error through actuations in the car.

### What are PID Controllers

A PID (Proportional-Integral-Derivative) controller is a control loop feedback mechanism widely used in industrial control systems and a variety of applications requiring continuously modulated control, including autonomous vehicles.

PID controllers work by calculating an error value as the difference between a desired setpoint and a measured process variable, then applying a correction based on proportional, integral, and derivative terms. The controller attempts to minimize the error over time by adjusting the control variable.

### Components of PID Control

A PID controller consists of three separate parameters:

**1. Proportional (P)**
- Responds directly to the current error
- Produces an output proportional to the present error value
- Formula: P_out = K_p × e(t)
- Larger K_p values generally mean faster response, but may lead to instability if too high
- P control alone often results in steady-state error (offset between desired and actual values)

**2. Integral (I)**
- Accounts for past values of the error
- Accumulates the error over time
- Formula: I_out = K_i × ∫e(t)dt
- Eliminates residual steady-state error that occurs with proportional-only control
- Can cause "integral windup" if the error persists for extended periods, requiring anti-windup mechanisms

**3. Derivative (D)**
- Predicts future behavior of the error
- Calculates the rate of change of the error
- Formula: D_out = K_d × d/dt e(t)
- Improves settling time and stability
- Sensitive to noise; may amplify high-frequency noise in the error signal

## How was the system tuned

Tuning involved the following steps:
1. Start with all gains at zero
2. Increase K_p until the system oscillates
3. Increase K_d to dampen oscillations
4. Add K_i to eliminate steady-state error
5. Repeat

### Steering Control

For lateral control, we implemented a steering controller that calculates the error between the vehicle's current heading and the desired heading toward the next waypoint. Key features of the steering controller include:

- **Error Calculation**: Determining the angle between the current position and the next point in the trajectory, then calculating the difference from the current yaw
- **Error Normalization**: Handling angle wrapping to ensure the controller always takes the shortest turning direction by normalizing the error to the range [-π, π]
- **Parameter Tuning**: Using moderately high proportional gain (Kp = 0.7) for responsive steering, small integral gain (Ki = 0.01) to correct persistent steering bias, and sufficient derivative gain (Kd = 0.2) to reduce oscillations during turns
- **Output Range**: Limiting steering commands to [-1.2, 1.2] to match the vehicle's physical steering limitations

### Throttle and Brake Control

For longitudinal control, we implemented a throttle controller that manages both acceleration and deceleration:

- **Error Calculation**: Computing the difference between the desired velocity (from trajectory points) and the current velocity
- **Throttle/Brake Splitting**: When the controller output is positive, it's applied as throttle; when negative, it's converted to brake commands
- **Parameter Tuning**: Using moderate proportional gain (Kp = 0.25) for smooth acceleration, small integral gain (Ki = 0.05) to eliminate steady-state error, and minimal derivative gain (Kd = 0.05) to prevent jerky responses
- **Output Range**: Limiting throttle and brake commands to [-1.0, 1.0] to match the vehicle's capabilities

