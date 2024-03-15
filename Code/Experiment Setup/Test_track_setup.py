import carla
from global_route_planner import GlobalRoutePlanner
import math


def calculate_left_right_points(x, y, direction, distance=1):
    """
    Calculate points to the left and right of a given point at a specified distance.

    :param x: X coordinate of the original point
    :param y: Y coordinate of the original point
    :param direction: Direction in degrees from the positive x-axis
    :param distance: Distance to the left and right points from the original point
    :return: A tuple containing the left and right points as tuples (left_point, right_point)
    """
    # Convert direction to radians
    direction_rad = math.radians(direction)

    # Calculate left and right directions in radians
    left_direction = direction_rad + math.pi / 2
    right_direction = direction_rad - math.pi / 2

    # Calculate offsets for left point
    left_x = x + distance * math.cos(left_direction)
    left_y = y + distance * math.sin(left_direction)

    # Calculate offsets for right point
    right_x = x + distance * math.cos(right_direction)
    right_y = y + distance * math.sin(right_direction)

    # Return the left and right points
    left_point = (left_x, left_y)
    right_point = (right_x, right_y)

    return left_point, right_point


def get_route_waypoints(route_planner, start_point, end_point):
    route_tuple_list = route_planner.trace_route(start_point, end_point)
    route_waypoints = [route_tuple[0] for route_tuple in route_tuple_list]

    return route_waypoints


def create_trajectory_points(map_object, distance_between_waypoints, points):
    route_planner = GlobalRoutePlanner(map_object, distance_between_waypoints)

    list_of_trajectory_waypoints = []
    for i in range(len(points) - 1):
        list_of_trajectory_waypoints.extend(
            [waypoint for waypoint in get_route_waypoints(route_planner, points[i], points[i + 1])])

    list_of_trajectory_points = [(waypoint.transform.location.x, waypoint.transform.location.y)
                                 for waypoint in list_of_trajectory_waypoints]

    return list_of_trajectory_points, list_of_trajectory_waypoints


def setup_test_track(carla_map, world, distance_between_trajectory_waypoints):
    map_points = [(waypoint.transform.location.x, waypoint.transform.location.y) for waypoint in
                  carla_map.generate_waypoints(1.0)]

    # CREATE TEST TRAJECTORY
    point_1 = carla.Location(58, 66.3, 0)
    point_2 = carla.Location(-41.5, 53.7, 0)
    point_3 = carla.Location(-25.8, -57.7, 0)
    point_4 = carla.Location(96.3, 106.9, 0)
    point_5 = carla.Location(-36.7, 127.1, 0)
    point_52 = carla.Location(-41.3, 97.1, 0)
    point_55 = carla.Location(-45, 91, 0)
    point_6 = carla.Location(-77.6, 16.5, 0)
    point_7 = carla.Location(97.5, 119.3, 0)
    point_8 = carla.Location(-22.7, -64.7, 0)
    point_9 = carla.Location(-48.8, -0.6, 0)
    point_list = [point_1, point_2, point_3, point_4, point_5, point_52, point_55,
                  point_6, point_7, point_8, point_9]

    (list_of_trajectory_points,
     list_of_trajectory_waypoints) = create_trajectory_points(carla_map,
                                                              distance_between_trajectory_waypoints,
                                                              point_list)

    # add cones and obstacles to test trajectory
    standard_blueprint_dict = {bp.id: bp for bp in world.get_blueprint_library().filter("static.prop.*")}

    warning_sign_object = None
    warning_sign_blueprint = standard_blueprint_dict["static.prop.trafficwarning"]
    warning_sign_position = carla.Transform(carla.Location(-41.5, 84, 0.1),
                                            carla.Rotation(0, 180, 0))

    try:
        warning_sign_object = world.spawn_actor(warning_sign_blueprint, warning_sign_position)
    except RuntimeError:
        pass

    cone_blueprint = standard_blueprint_dict["static.prop.constructioncone"]
    cone_object_list = []
    distances = [3, 3.5]
    cone_locations = []
    for distance in distances:
        left_point, right_point = calculate_left_right_points(list_of_trajectory_waypoints[0].transform.location.x,
                                                              list_of_trajectory_waypoints[0].transform.location.y,
                                                              list_of_trajectory_waypoints[0].transform.rotation.yaw,
                                                              distance=distance)
        cone_locations.append(left_point)
        cone_locations.append(right_point)

        left_point, right_point = calculate_left_right_points(list_of_trajectory_waypoints[-1].transform.location.x,
                                                              list_of_trajectory_waypoints[-1].transform.location.y,
                                                              list_of_trajectory_waypoints[-1].transform.rotation.yaw,
                                                              distance=distance)
        cone_locations.append(left_point)
        cone_locations.append(right_point)

    start_segment = (cone_locations[0], cone_locations[1])
    end_segment = (cone_locations[-2], cone_locations[-1])
    for point in cone_locations:
        cone_position = carla.Transform(carla.Location(point[0], point[1], 0.1),
                                        carla.Rotation(0, 180, 0))
        try:
            cone_object = world.spawn_actor(cone_blueprint, cone_position)
            cone_object_list.append(cone_object)
        except RuntimeError:
            pass

    return (warning_sign_object, cone_object_list, list_of_trajectory_points, list_of_trajectory_waypoints,
            start_segment, end_segment, map_points)


def draw_trajectory_line(world, list_of_trajectory_waypoints, color, life_time):
    for i in range(0, len(list_of_trajectory_waypoints) - 1, 10):
        start = list_of_trajectory_waypoints[i].transform.location
        end = list_of_trajectory_waypoints[i + 1].transform.location
        world.debug.draw_line(start, end, thickness=0.1, color=color, life_time=life_time,
                              persistent_lines=True)
