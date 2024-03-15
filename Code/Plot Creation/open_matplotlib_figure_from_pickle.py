import pickle
import matplotlib.pyplot as plt
import numpy as np
import os


def make_same_length(times, list_of_values):
    diff = len(times) - len(list_of_values)
    if diff > 0:
        times = times[:-diff]
    elif diff < 0:
        list_of_values = list_of_values[:diff]

    return times, list_of_values


def open_figure(test_number):
    test_directory_name = None
    directory = "/home/danny/Documents/University/Bachelorarbeit/Test_Results/Michal"
    all_test_directories = [f.path for f in os.scandir(directory) if f.is_dir()]
    for directory in all_test_directories:
        if f"T{test_number}" in directory:
            test_directory_name = directory

    if test_directory_name:
        with open(f"{test_directory_name}/Pickle_files/map.pkl", 'rb') as file:
            (map_waypoints, trajectory_points, vehicle_waypoints, delay) = pickle.load(file)
        plot_map(map_waypoints, trajectory_points, vehicle_waypoints, delay)

        """with open(f"{test_directory_name}/Pickle_files/steering.pkl", 'rb') as file:
            (steering_angle_list, speed, delay, test_time) = pickle.load(file)
            speed = int(round(speed, 1))
        steering_plot(steering_angle_list, speed, delay, test_time)"""

        """with open(f"{test_directory_name}/Pickle_files/lane_deviation.pkl", 'rb') as file:
            (lane_deviation_list, speed, delay, test_time) = pickle.load(file)
            speed = int(round(speed, 1))
        lane_deviation_plot(lane_deviation_list, speed, delay, test_time)"""

    else:
        print(f"No test with number {test_number} found.")


def plot_map(map_points, trajectory_points, vehicle_waypoints, delay):
    x_map_coordinates = []
    y_map_coordinates = []
    for point in map_points:
        x_map_coordinates.append(point[0])
        y_map_coordinates.append(point[1])

    x_coordinates = []
    y_coordinates = []
    for waypoint in trajectory_points:
        x_coordinates.append(waypoint[0])
        y_coordinates.append(waypoint[1])

    x_vehicle_coordinates = []
    y_vehicle_coordinates = []
    for point in vehicle_waypoints:
        x_vehicle_coordinates.append(point[0])
        y_vehicle_coordinates.append(point[1])

    plt.axis("equal")
    plt.scatter(x_map_coordinates, y_map_coordinates, s=0.05, color="grey")
    plt.scatter(x_coordinates, y_coordinates, s=0.2, color="blue")
    #  plt.scatter(x_vehicle_coordinates, y_vehicle_coordinates, s=0.1, color="blue")
    plt.scatter([56],[66], s=50, color="lightgreen", marker=",", label="Start")
    plt.scatter([-49],[0], s=50, color="red", marker=",", label="Finish")
    plt.scatter([-41],[90], s=30, color="orange", marker=",", label="Obstacle")


    plt.legend()
    plt.gca().invert_yaxis()
    plt.show()


def steering_plot(steering_angle_list, speed, delay, test_time):
    times = np.arange(0, test_time, test_time / len(steering_angle_list))

    tele_operator_steering_angle_list = [angle_tuple[0] for angle_tuple in steering_angle_list]
    controller_steering_angle_list = [angle_tuple[1] for angle_tuple in steering_angle_list]

    if len(times) != len(tele_operator_steering_angle_list):
        times, tele_operator_steering_angle_list = make_same_length(times, tele_operator_steering_angle_list)

    if len(times) != len(controller_steering_angle_list):
        times, controller_steering_angle_list = make_same_length(times, controller_steering_angle_list)

    plt.plot(times, tele_operator_steering_angle_list, linestyle='-', label="Teleoperator")
    plt.plot(times, controller_steering_angle_list, linestyle='-', label="Controller")

    plt.title(f'Speed: {speed} m/s, delay: {delay} ms')
    plt.xlabel('Time (s)')
    plt.ylabel('Steering input')
    plt.axis('equal')
    plt.legend()
    plt.grid(True)
    plt.show()


def lane_deviation_plot(lane_deviation_list, speed, delay, test_time):
    total_lane_deviation = 0
    for lane_deviation in lane_deviation_list:
        total_lane_deviation = total_lane_deviation + abs(lane_deviation)

    average_lane_deviation = total_lane_deviation / len(lane_deviation_list)

    times = np.arange(0, test_time, test_time / len(lane_deviation_list))

    if len(times) != len(lane_deviation_list):
        times, lane_deviation_list = make_same_length(times, lane_deviation_list)

    plt.plot(times, lane_deviation_list, linestyle='-', label="lane deviation")

    plt.title(f'Speed: {speed}, delay: {delay}ms, avg lane dev: {average_lane_deviation}')
    plt.xlabel('Time (seconds)')
    plt.ylabel('lane deviation')
    plt.axis('equal')
    plt.legend()
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    open_figure(46)
