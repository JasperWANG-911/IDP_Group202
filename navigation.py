#!/usr/bin/env python3
import time
import machine
from Pathfinder import get_edge_direction, get_next_node, compute_turn_type, check_node_sensor
from turning import turn_until_shift, turn_90
from line_sensor import LineSensors
from orientation_control import OrientationController
import collection_dropoff as cd

class Navigation:
    def __init__(self, motors, target_route=None, base_speed=75, pid_params=(20, 0.5, 10)):
        """
        Initialize the Navigation class.
        Args:
            motors: MotorPair instance controlling the robot.
            target_route (list): Target nodes (default: ['X1', 'X2', 'X3', 'X4', 'RY', 'BG']).
            base_speed (int): Default base speed.
            pid_params (tuple): PID parameters (k_p, k_i, k_d).
        """
        self.motors = motors
        self.sensor_instance = LineSensors()
        self.target_route = target_route if target_route is not None else ['X1', 'X2', 'X3', 'X4', 1]
        k_p, k_i, k_d = pid_params
        self.orientation_controller = OrientationController(base_speed=base_speed, k_p=k_p, k_i=k_i, k_d=k_d)
        self.led = machine.Pin(14, machine.Pin.OUT)
        # Use previous 0/1/2/3 for direction.
        self.current_orientation = 0
        self.actuator = None
        self.TOF_sensor = None
        self.colour_sensor = None

    def flash_led(self, flashes=1, duration=0.5):
        """Flash the LED a specified number of times."""
        for _ in range(flashes):
            self.led.value(1)
            time.sleep(duration)
            self.led.value(0)
            time.sleep(duration)

    def controlled_move_forward(self, duration, update_interval=0.05):
        """Drive forward for the given duration while updating the controller."""
        start_time = time.time()
        while time.time() - start_time < duration:
            self.orientation_controller.update()
            time.sleep(update_interval)

    def controlled_move_backward(self, duration, update_interval=0.05):
        """Drive backward for the given duration while updating reverse control."""
        start_time = time.time()
        while time.time() - start_time < duration:
            self.orientation_controller.update_reverse()
            time.sleep(update_interval)

    def run(self, actuator, TOF_sensor, colour_sensor):
        """
        Main navigation loop.
        (a) At the start, drive forward until the start line (detected by side sensor) is seen.
        (b) Then, for each edge of the route, the system continuously updates the controller and sensor.
            It waits until a cross is detected (one side sensor active continuously for 0.3s).
            Only then are the graph computations (next node, desired direction, turn type) performed.
            If a left/right turn is needed, the turn is executed.
        Returns:
            List of visited nodes.
        """
        self.actuator = actuator
        self.TOF_sensor = TOF_sensor
        self.colour_sensor = colour_sensor
        current_node = 1
        next_node = 2
        visited = [current_node]
        
        # Start: drive until start line detected.
        print("Searching for start line...")
        while True:
            sp = self.sensor_instance.read_all()
            if sp.get('left_side') == 1 or sp.get('right_side') == 1:
                print("Start line detected at node 1.")
                self.led.value(1)
                self.controlled_move_forward(0.5)
                break
            else:
                self.orientation_controller.update()
                time.sleep(0.05)
        
        target_index = 0
        num_targets = len(self.target_route)
        target = self.target_route[target_index]

        while target_index < num_targets:
            
            '''
            if current_node == target:
                print(f"Reached target node: {target}")
                
                self.flash_led(flashes=1, duration=0.2)
                # At marking nodes, if a marking line is detected, pause and reverse.
                if target in ['X1', 'X2', 'X3', 'X4', 'RY', 'BG']:
                    sp = self.sensor_instance.read_all()
                    print("Sensor data at marking:", sp)
                    if sp.get('left_side') == 1 or sp.get('right_side') == 1:
                        print(f"Marking line detected at {target}. Pausing for 3 seconds.")
                        self.orientation_controller.stop()
                        time.sleep(3)
                    print("Executing reverse maneuver to leave marking node.")
                    self.controlled_move_backward(0.5)
                if target == 1:
                    print("Finish li
                    
                    
                    ne detected. Moving forward a short distance and stopping.")
                    self.controlled_move_forward(0.5)
                    break
                target_index += 1
                continue
            '''
            # Wait until a cross is detected by sensor pattern.
            cross_stable_start = None
            print("Waiting for cross detection...")
            while True:
                self.orientation_controller.update()
                sp = self.sensor_instance.read_all()
                # Print sensor data for debugging.
                print("Sensor data:", sp)
                if sp.get('left_side') == 1 or sp.get('right_side') == 1:
                    #if cross_stable_start is None:
                        #cross_stable_start = time.time()
                    #elif time.time() - cross_stable_start >= 0.05:
                    print("Cross detected: side sensor active for 0.05s.")
                    current_node = next_node
                    self.orientation_controller.stop()
                    if current_node == target:
                        if current_node in ['X1', 'X2', 'X3', 'X4']:
                            target_index += 1
                            print('Collecting parcel')
                            target = cd.collection(self.motors, self.actuator, self.TOF_sensor, self.colour_sensor)
                            self.controlled_move_backward(0.2)
                        elif current_node in ['RY', 'BG']:
                            cd.drop_off(self.motors, actuator, TOF_sensor)
                            target = self.target_route[target_index]
                            self.controlled_move_backward(0.2)
                        elif current_node == 1:
                            self.controlled_move_forward(1)
                            target_index = 10000000
                    break
                #else:
                    #cross_stable_start = None
                time.sleep(0.05)
            
            # Now that a cross is detected, compute the graph-related information.
            next_node = get_next_node(current_node, target)
            print(f"Graph computation: Current node: {current_node}, Next node: {next_node}, Target: {target}")
            edge_dir = get_edge_direction(current_node, next_node)
            if edge_dir is None:
                print('no avaliable edge')
                desired_direction = 0  # Default (North)
            else:
                mapping = {'N': 0, 'E': 1, 'S': 2, 'W': 3}
                desired_direction = mapping.get(edge_dir, 0)

            
            turn_type = compute_turn_type(self.current_orientation, desired_direction)
            print(f"Graph computation: Current Orientation: {self.current_orientation}, Desired: {desired_direction}, Turn: {turn_type}")
            check_node_sensor(self.sensor_instance, current_node)
            
            # Execute turning only if turn_type is left/right and cross was detected.
            if turn_type in ['left', 'right']:
                self.orientation_controller.stop()
                time.sleep(0.1)
                turn_time = 2.2
                '''
                if (sp.get('center_left') == 1 and sp.get('center_right') == 0):
                    orientation_discrepancy = 'right'
                elif (sp.get('center_left') == 0 and sp.get('center_right') == 1):
                    orientation_discrepancy = 'left'
                else:
                    orientation_discrepancy = None
                
                if orientation_discrepancy != None:
                    if orientation_discrepancy == turn_type:
                        turn_time -= 0.3
                    else:
                        turn_time += 0.3
                '''
                turn_until_shift(self.orientation_controller, self.sensor_instance, turn_type=turn_type, timeout= 1.8, turning_sensitivity=0)
                print(f"Executed {turn_type} turn at cross.")
                self.current_orientation = desired_direction
                self.controlled_move_forward(0.5)
            elif turn_type == 'rear':
                print("Executing reverse move (without turning) to reach next node.")
                self.controlled_move_backward(0.6)
                # Increase sensor sampling frequency during reverse
                sp = self.sensor_instance.read_all()
                while sp.get('left_side') == 0 and sp.get('right_side') == 0:
                    self.orientation_controller.update_reverse()
                    sp = self.sensor_instance.read_all()
                    time.sleep(0.05)  # reduced sleep interval for more frequent updates
                self.orientation_controller.stop()

                #turn_90(self.orientation_controller, self.sensor_instance, angle=180, turn_type='right', turn_time=turn_time)      
                #self.current_orientation = desired_direction         
            else:
                self.controlled_move_forward(1.5)
                print("No turning required (straight).")
            
            #self.controlled_move_forward(0.5)
            #current_node = next_node
            if visited[-1] != current_node:
                visited.append(current_node)
            time.sleep(0.01)
        
        self.led.value(0)
        print("Navigation complete. All target nodes reached.")
        return visited
    

if __name__ == "__main__":
    from motor import Motor1, Motor2, MotorPair
    left_motor = Motor2()
    right_motor = Motor1()
    motors = MotorPair(left_motor, right_motor)
    nav = Navigation(motors, target_route=['X1', 'X2', 'X3', 'X4', 'RY', 'BG'], base_speed=75, pid_params=(20, 0.5, 10))
    visited_nodes = nav.run()
    print("Visited nodes:", visited_nodes)
