# Navigation.py
import time
from Pathfinder import get_edge_direction, get_next_node, compute_turn_type, check_node_sensor
from turning import turn_until_shift, perform_reverse_turn
from line_sensor import LineSensors
from orientation_control import OrientationController

class Navigation:
    def __init__(self, motors, target_route=None, base_speed=75, pid_params=(20, 0.5, 10)):
        """
        Initialize the Navigation class with tunable parameters.
        Args:
            motors: MotorPair instance controlling the robot.
            target_route (list): A list of target nodes (default: ['X1', 'X2', 'X3', 'X4', 'RY', 'BG']).
            base_speed (int): Default base speed for orientation control.
            pid_params (tuple): PID parameters as (k_p, k_i, k_d).
        """
        self.motors = motors
        self.sensor_instance = LineSensors()
        self.target_route = target_route if target_route is not None else ['X1', 'X2', 'X3', 'X4', 'RY', 'BG']
        k_p, k_i, k_d = pid_params
        self.orientation_controller = OrientationController(base_speed=base_speed, k_p=k_p, k_i=k_i, k_d=k_d)

    def controlled_move_forward(self, duration, update_interval=0.1):
        """
        Drives forward for the specified duration while continuously updating orientation control.
        """
        start_time = time.time()
        while time.time() - start_time < duration:
            self.orientation_controller.update()  # continuously update PID control
            time.sleep(update_interval)

    def controlled_move_backward(self, duration, update_interval=0.1):
        """
        Drives backward for the specified duration while continuously updating reverse orientation control.
        """
        start_time = time.time()
        while time.time() - start_time < duration:
            self.orientation_controller.update_reverse()  # continuously update reverse PID control
            time.sleep(update_interval)

    def run(self):
        """
        Main navigation loop integrating the pathfinder and turning modules.
        Special rules:
          a. At node 1 (start/finish), if all sensors are active, drive forward slightly.
          b. At marking nodes, if all sensors are active, pause for 3 seconds.
          c. For reverse moves, drive backward continuously (with reverse updates) until reaching the next node.
             Then stop and let the loop decide the next turn.
        Returns:
            List of visited nodes.
        """
        current_node = 1
        visited = [current_node]
        target_index = 0
        num_targets = len(self.target_route)
        
        # At start: if all sensors are active at node 1, assume start line detected.
        sp = self.sensor_instance.read_all()
        if all(val == 1 for val in sp.values()):
            print("Start line detected at node 1.")
            self.controlled_move_forward(0.5)

        while target_index < num_targets:
            target = self.target_route[target_index]
            if current_node == target:
                print(f"Reached target node: {target}")
                # At marking nodes, pause and execute reverse maneuver.
                if target in ['X1', 'X2', 'X3', 'X4', 'RY', 'BG']:
                    sp = self.sensor_instance.read_all()
                    if all(val == 1 for val in sp.values()):
                        print(f"Marking line detected at {target}. Pausing for 3 seconds.")
                        time.sleep(3)
                    print("Executing reverse maneuver to leave marking node.")
                    # Reverse maneuver without turning:
                    self.controlled_move_backward(0.5)
                # For finish (node 1), drive forward slightly and stop.
                if target == 1:
                    print("Finish line detected. Moving forward a short distance and stopping.")
                    self.controlled_move_forward(0.5)
                    break
                target_index += 1
                continue

            next_node = get_next_node(current_node, target)
            print(f"Current node: {current_node}, Next node: {next_node}, Target: {target}")

            edge_dir = get_edge_direction(current_node, next_node)
            if edge_dir is None:
                desired_direction = 0  # Default (North)
            else:
                mapping = {'N': 0, 'E': 1, 'S': 2, 'W': 3}
                desired_direction = mapping.get(edge_dir, 0)

            # For simulation, assume current orientation is 0 (North).
            current_orientation = 0
            turn_type = compute_turn_type(current_orientation, desired_direction)
            print(f"Current Orientation: {current_orientation}, Desired: {desired_direction}, Turn: {turn_type}")

            check_node_sensor(self.sensor_instance, current_node)

            if turn_type == 'straight':
                self.controlled_move_forward(0.5)
                current_node = next_node
                if visited[-1] != current_node:
                    visited.append(current_node)
            elif turn_type in ['left', 'right']:
                turn_until_shift(self.motors, self.sensor_instance, self.orientation_controller,
                                  turn_type, increment=0.1, timeout=3)
                sp = self.sensor_instance.read_all()
                if all(val == 1 for val in sp.values()):
                    print(f"Sensor pattern confirmed after {turn_type} turn (marking line detected).")
                    self.controlled_move_forward(0.5)
                    current_node = next_node
                    if visited[-1] != current_node:
                        visited.append(current_node)
                else:
                    print(f"Sensor pattern incorrect after {turn_type} turn; not updating node.")
            elif turn_type == 'rear':
                print("Executing reverse move (without turning) to reach next node.")
                self.controlled_move_backward(0.5)
                current_node = next_node
                if visited[-1] != current_node:
                    visited.append(current_node)
                print("Reverse move complete; new node reached.")
            time.sleep(0.1)
        
        print("Navigation complete. All target nodes reached.")
        return visited

if __name__ == "__main__":
    # For testing when run directly.
    from motor import Motor1, Motor2, MotorPair
    left_motor = Motor2()
    right_motor = Motor1()
    motors = MotorPair(left_motor, right_motor)
    nav = Navigation(motors, target_route=['X1', 'X2', 'X3', 'X4', 'RY', 'BG'], base_speed=75, pid_params=(20, 0.5, 10))
    visited_nodes = nav.run()
    print("Visited nodes:", visited_nodes)
