# Define the distance per move (each grid cell is ~0.22 m)
DISTANCE_PER_MOVE = 0.22

class Odometry:
    """
    Odometry class maintains the robot's position and orientation,
    and provides methods to update them based on movement commands.
    """
    def __init__(self):
        # Current position as (x, y) in meters
        self.current_position = (0.0, 0.0)
        # Vehicle orientation: 0 = North, 1 = East, 2 = South, 3 = West
        self.vehicle_orientation = 0

    def get_new_position(self, position, orientation, distance=DISTANCE_PER_MOVE):
        """
        Calculate a new position given a starting position, orientation, and distance.
        Orientation: 0 = North, 1 = East, 2 = South, 3 = West.
        """
        x, y = position
        if orientation == 0:      # North: increase y
            return (x, y + distance)
        elif orientation == 1:    # East: increase x
            return (x + distance, y)
        elif orientation == 2:    # South: decrease y
            return (x, y - distance)
        elif orientation == 3:    # West: decrease x
            return (x - distance, y)
        return position

    def update_position(self, move_type):
        """
        Update the current position based on movement type.
        'front': move forward based on current orientation.
        'rear': move backward (i.e., opposite of current orientation).
        """
        if move_type == 'front':
            self.current_position = self.get_new_position(self.current_position, self.vehicle_orientation)
        elif move_type == 'rear':
            back_orientation = (self.vehicle_orientation + 2) % 4
            self.current_position = self.get_new_position(self.current_position, back_orientation)
        else:
            raise ValueError("Invalid move type. Use 'front' or 'rear'.")

    def update_orientation(self, turn_direction):
        """
        Update the vehicle's orientation based on turn direction.
        'left': turn left (counter-clockwise).
        'right': turn right (clockwise).
        """
        if turn_direction == 'left':
            self.vehicle_orientation = (self.vehicle_orientation - 1) % 4
        elif turn_direction == 'right':
            self.vehicle_orientation = (self.vehicle_orientation + 1) % 4
        else:
            raise ValueError("Invalid turn direction. Use 'left' or 'right'.")

    def update_after_turn_move(self, turn_direction, move_type):
        """
        Update odometry after a turn and subsequent move.
        This method first updates the orientation (so the new direction takes effect),
        and then updates the position based on the new orientation.
        """
        self.update_orientation(turn_direction)
        self.update_position(move_type)

    def __str__(self):
        return f"Position: {self.current_position}, Orientation: {self.vehicle_orientation}"
