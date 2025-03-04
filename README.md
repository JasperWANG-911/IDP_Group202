
This project implements a navigation system for a Raspberry Pi–based robot using modular code. The system integrates graph-based path planning, sensor-based orientation control (using PID), and turning logic to guide the robot through a predetermined route. The design is fully tunable from the main script.

## Directory Structure

. ├── graph_classes.py # Graph data structure and Dijkstra algorithm ├── line_sensor.py # Module for reading line sensor data (center sensors) ├── motor.py # Motor control classes (Motor1, Motor2, and MotorPair) ├── Pathfinder.py # Functions for extracting edge directions, next node, and turn computation ├── turning.py # Functions for performing turning maneuvers (with orientation control updates) ├── orientation_control.py # OrientationController class implementing PID-based control ├── navigation.py # Navigation class that integrates all modules for route execution └── main.py # Main entry point; instantiates motors and Navigation with tunable parameters

markdown
复制

## Module Descriptions

- **graph_classes.py**  
  Defines the `Graph`, `Node`, and `Edge` classes along with a Dijkstra implementation to compute the shortest path between nodes in a map. This module is used by the path planning functions.

- **line_sensor.py**  
  Contains the `LineSensors` class which configures and reads from the two center line sensors. These readings are used for PID-based orientation control.

- **motor.py**  
  Provides classes for motor control:
  - `Motor1`: Controls one motor (used as the right motor).
  - `Motor2`: Controls the other motor (used as the left motor).
  - `MotorPair`: Encapsulates coordinated control of the two motors (forward, backward, turning).

- **Pathfinder.py**  
  Offers helper functions:
  - `get_edge_direction()`: Returns the cardinal direction between nodes.
  - `get_next_node()`: Uses Dijkstra's algorithm to compute the next node along the shortest path.
  - `compute_turn_type()`: Determines whether to go straight, turn left/right, or reverse based on the current and desired orientations.
  - `check_node_sensor()`: Verifies that the sensor readings at a node match the expected pattern.

- **turning.py**  
  Contains functions to execute turning maneuvers:
  - `turn_until_shift()`: Increments turning commands while periodically calling the orientation controller’s update so that the robot’s “nose” (center sensors) remains aligned.
  - `perform_reverse_turn()`: Executes a reverse maneuver with orientation correction.

- **orientation_control.py**  
  Implements the `OrientationController` class which uses a PID controller to adjust motor speeds based on errors computed from the two center sensors. It provides methods for both forward (`update()`) and reverse (`update_reverse()`) control.

- **navigation.py**  
  Wraps the overall navigation logic into a `Navigation` class that:
  - Accepts tunable parameters (target route, base speed, PID parameters) from the main script.
  - Integrates sensor readings, path planning, turning maneuvers, and orientation control to execute a predetermined route.
  - Implements special rules:
    - At node 1 (start/finish), if all sensors are active, the robot drives forward slightly.
    - At marking nodes (e.g., X1, X2, X3, X4, RY, BG), it pauses for 3 seconds and executes a reverse maneuver.
    - For reverse moves, the robot simply drives backward (without turning), then stops and performs a tuning phase before moving forward.

- **main.py**  
  The main entry point that:
  - Instantiates the motor pair.
  - Reads tunable parameters (target route, base speed, PID parameters) from variables.
  - Creates an instance of the `Navigation` class with these parameters.
  - Runs the navigation routine.

## Flowchart of Code Structure

```mermaid
flowchart TD
    A[main.py] --> B[MotorPair<br/>(Motor1 & Motor2)]
    A --> C[Navigation Class]
    C --> D[Pathfinder Functions<br/>(get_edge_direction, get_next_node, compute_turn_type)]
    C --> E[Turning Module<br/>(turn_until_shift, perform_reverse_turn)]
    C --> F[OrientationController<br/>(PID, sensor readings)]
    F --> G[line_sensor.py<br/>(LineSensors)]
    
    subgraph "Navigation Flow"
       C --> H[Navigation Loop]
       H --> I{Is current node target?}
       I -- Yes --> J[Mark node: pause & reverse maneuver<br/>(if marking node)]
       I -- No --> K[Determine next node via Dijkstra]
       K --> L[Compute turn type]
       L -- "straight" --> M[Update orientation (forward) and move forward]
       L -- "left/right" --> N[Call turn_until_shift() with orientation updates]
       L -- "rear" --> O[Move backward, tune (update orientation), then move forward]
    end
Usage Instructions
Configuration:
Modify the tunable parameters in main.py:

target_route: List of target nodes (default: ['X1', 'X2', 'X3', 'X4', 'RY', 'BG']).
base_speed: Default speed (e.g., 75).
pid_params: PID constants as a tuple (k_p, k_i, k_d).
Running the System:
Execute main.py on your Raspberry Pi. The main script initializes the motors, creates a Navigation instance with the provided parameters, and runs the navigation routine.

Tuning:
Adjust the PID parameters and base speed as needed to match your hardware dynamics.

Summary
Graph & Pathfinder:
Provide route planning via Dijkstra’s algorithm.
Orientation Control:
Uses PID to adjust motor speeds based on sensor feedback.
Turning & Navigation:
Integrates turning maneuvers (with continuous orientation updates) and reverse moves with a tuning phase.
Main Script:
Configurable target route, base speed, and PID parameters make it easy to tune the system.
