from Pathfinder import get_edge_direction, get_next_node, compute_turn_type, check_node_sensor
# Now that a cross is detected, compute the graph-related information.
next_node = get_next_node(1, 'X1')
print(f"Graph computation: Current node: {1}, Next node: {next_node}, Target: {'X1'}")
edge_dir = get_edge_direction(1, next_node)
if edge_dir is None:
    print('no avaliable edge')
    desired_direction = 0  # Default (North)
else:
    mapping = {'N': 0, 'E': 1, 'S': 2, 'W': 3}
    desired_direction = mapping.get(edge_dir, 0)
    
print(desired_direction)