# odometry.py
DISTANCE_PER_MOVE = 0.22  # meters per move

def get_new_position(position, orientation, distance=DISTANCE_PER_MOVE):
    x, y = position
    if orientation == 0:  # North
        return (x, y + distance)
    elif orientation == 1:  # East
        return (x + distance, y)
    elif orientation == 2:  # South
        return (x, y - distance)
    elif orientation == 3:  # West
        return (x - distance, y)
    return position

def update_position(current_position, move_type, vehicle_orientation):
    if move_type == 'front':
        return get_new_position(current_position, vehicle_orientation)
    elif move_type == 'rear':
        back_orientation = (vehicle_orientation + 2) % 4
        return get_new_position(current_position, back_orientation)
