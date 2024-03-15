import numpy as np


def point_line_distance_and_side(point, start, end):
    """Calculate the distance from a point to a line segment and determine the side."""
    px = point[0]
    py = point[1]
    sx, sy = start
    ex, ey = end
    dx = ex - sx
    dy = ey - sy
    if dx == 0 and dy == 0:
        return np.sqrt((px - sx) ** 2 + (py - sy) ** 2), 0  # Segment is a point

    t = max(0, min(1, ((px - sx) * dx + (py - sy) * dy) / (dx ** 2 + dy ** 2)))
    closest_point = sx + t * dx, sy + t * dy
    distance = np.sqrt((px - closest_point[0]) ** 2 + (py - closest_point[1]) ** 2)
    cross_product = dx * (py - sy) - dy * (px - sx)
    side = np.sign(cross_product)

    return distance, side


def get_lane_deviation(point, trajectory):
    min_distance = None
    min_lane_deviation = None

    for i in range(len(trajectory) - 1):
        segment_start, segment_end = trajectory[i], trajectory[i + 1]

        distance, side = point_line_distance_and_side(point, segment_start, segment_end)

        if min_distance is None or distance < min_distance:
            min_distance = distance
            min_lane_deviation = distance * side

    return min_lane_deviation
