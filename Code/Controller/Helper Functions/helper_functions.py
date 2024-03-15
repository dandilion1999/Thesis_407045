import numpy as np
import math


def calculate_distance(point1, point2):
    return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)


def get_change_in_value(current_value, previous_value):
    # shift steering angles into positive range to make calculation easier
    shifted_steering_angle = current_value + 1
    shifted_previous_steering_angle = previous_value + 1

    change = shifted_steering_angle - shifted_previous_steering_angle

    return change


def dampen_value(previous_value, current_value, max_change):
    change = current_value - previous_value

    if abs(change) > max_change:
        # Determine the direction of the change and apply the maximum allowed change
        change_direction = change / abs(change)  # Will be 1 for increase, -1 for decrease
        adjusted_value = previous_value + change_direction * max_change
    else:
        adjusted_value = current_value

    return adjusted_value


def get_heading_error(heading, desired_heading):
    heading_component_x = round(np.cos(heading), 3)
    heading_component_y = round(np.sin(heading), 3)

    desired_x = round(np.cos(desired_heading), 3)
    desired_y = round(np.sin(desired_heading), 3)

    dot_product = desired_x * heading_component_x + desired_y * heading_component_y
    # Calculate the determinant (or "cross product" for 2D vectors)
    determinant = desired_x * heading_component_y - heading_component_x * desired_y

    # Calculate the angle between the vectors
    heading_error = 1 - dot_product

    # Determine the sign of the angle based on the determinant's value
    if determinant < 0:
        heading_error = -heading_error

    return heading_error


def ccw(A, B, C):
    """Check if points A, B, and C are listed in counter-clockwise order."""
    return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])


def intersect(A, B, C, D):
    """Check if line segments AB and CD intersect."""
    return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)


def crossed_line(curr_point, prev_point, line):
    return intersect(prev_point, curr_point, line[0], line[1])


def plot_points_and_directions(directed_points):

    import matplotlib.pyplot as plt
    for i, (x, y, degrees) in enumerate(directed_points):
        # Convert degrees to radians and calculate dx, dy for the arrow
        radians = math.radians(degrees)
        dx = math.cos(radians)
        dy = math.sin(radians)

        # Plotting the arrow for each point
        plt.arrow(x, y, dx, dy, head_width=0.01, head_length=0.05, fc='blue', ec='blue')

        # Extracting x and y coordinates to plot points
    x_coords, y_coords = zip(*[(x, y) for x, y, _ in directed_points])
    plt.plot(x_coords, y_coords, 'ro', markersize=1)  # 'ro' for red circles

    # Set plot labels and grid
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.grid(True)
    plt.axis('equal')  # Equal aspect ratio ensures that pie chart is circular.
    plt.title('Points and Directed Angles')

    plt.show()


def calculate_lateral_acceleration(a_total, heading_angle_deg):
    """
    Calculate the lateral acceleration of a vehicle.
    
    Parameters:
    - a_total: Total acceleration of the vehicle (magnitude) in m/s^2.
    - heading_angle_deg: Heading angle of the vehicle in degrees, measured clockwise from north.
    
    Returns:
    - Lateral acceleration in m/s^2.
    """

    heading_angle_rad = math.radians(heading_angle_deg)
    # a_lateral = a_total * math.cos(heading_angle_rad)
    a_lateral = a_total * math.sin(heading_angle_rad)

    return a_lateral
