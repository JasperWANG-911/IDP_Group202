
## Module Descriptions

### graph_classes.py
Defines the `Graph`, `Node`, and `Edge` classes along with a Dijkstra implementation to compute the shortest path between nodes in a map. This module is used by the path planning functions.

### line_sensor.py
Contains the `LineSensors` class which configures and reads from the two center line sensors. These readings are used for PID-based orientation control.

### motor.py
Provides classes for motor control:
- **Motor1**: Controls one motor (used as the right motor).
- **Motor2**: Controls the other motor (used as the left motor).
- **MotorPair**: Encapsulates coordinated control of the two motors (forward, backward, turning).

### Pathfinder.py
Offers helper functions:
- `get_edge_direction()`: Returns the cardinal direction between nodes.
- `get_next_node()`: Uses Dijkstra's algorithm to compute the next node along the shortest path.
- `compute_turn_type()`: Determines whether to go straight, turn left/right, or reverse based on the current and desired orientations.
- `check_node_sensor()`: Verifies that the sensor readings at a node match the expected pattern.

### turning.py
Contains functions to execute turning maneuvers:
- `turn_until_shift()`: Increments turning commands while periodically calling the orientation controller’s update so that the robot’s "nose" (center sensors) remains aligned.
- `perform_reverse_turn()`: Executes a reverse maneuver with orientation correction.

### orientation_control.py
Implements the `OrientationController` class which uses a PID controller to adjust motor speeds based on errors computed from the two center sensors. It provides methods for both forward (`update()`) and reverse (`update_reverse()`) control.

### navigation.py
Wraps the overall navigation logic into a `Navigation` class that:
- Accepts tunable parameters (target route, base speed, PID parameters) from the main script.
- Integrates sensor readings, path planning, turning maneuvers, and orientation control to execute a predetermined route.
- Implements special rules:
  - At node 1 (start/finish), if all sensors are active, the robot drives forward slightly.
  - At marking nodes (e.g., X1, X2, X3, X4, RY, BG), it pauses for 3 seconds and executes a reverse maneuver.
  - For reverse moves, the robot simply drives backward (without turning), then stops and performs a tuning phase before moving forward.

### main.py
The main entry point that:
- Instantiates the motor pair.
- Reads tunable parameters (target route, base speed, PID parameters) from variables.
- Creates an instance of the `Navigation` class with these parameters.
- Runs the navigation routine.

