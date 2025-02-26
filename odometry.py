# Define the distance per move (each grid cell is ~0.22 m)
DISTANCE_PER_MOVE = 0.22

# Odometry class holds the robot's position and orientation and provides update methods
class Odometry:
    def __init__(self):
        self.current_position = (0.0, 0.0)  # in meters
        self.vehicle_orientation = 0        # 0 = North, 1 = East, 2 = South, 3 = West

    def get_new_position(self, position, orientation, distance=DISTANCE_PER_MOVE):
        x, y = position
        if orientation == 0:      # North (+y)
            return (x, y + distance)
        elif orientation == 1:    # East (+x)
            return (x + distance, y)
        elif orientation == 2:    # South (-y)
            return (x, y - distance)
        elif orientation == 3:    # West (-x)
            return (x - distance, y)
        return position

    def update_position(self, move_type):
        # Update position based on movement direction (front or rear)
        if move_type == 'front':
            self.current_position = self.get_new_position(self.current_position, self.vehicle_orientation)
        elif move_type == 'rear':
            back_orientation = (self.vehicle_orientation + 2) % 4
            self.current_position = self.get_new_position(self.current_position, back_orientation)

    def update_orientation(self, turn_direction):
        # Update orientation based on turn direction (left or right)
        if turn_direction == 'left':
            self.vehicle_orientation = (self.vehicle_orientation - 1) % 4
        elif turn_direction == 'right':
            self.vehicle_orientation = (self.vehicle_orientation + 1) % 4
