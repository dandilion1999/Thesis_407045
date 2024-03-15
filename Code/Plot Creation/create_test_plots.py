import pickle
import matplotlib.pyplot as plt
import math
import numpy as np

import os
import subprocess


def count_threshold_crossings(values, threshold):
    crossings = 0

    # Start with the second value to compare it with the first one
    for i in range(1, len(values)):
        # Check for upward crossing
        if values[i - 1] < threshold < values[i]:
            crossings += 1
        # Check for downward crossing
        elif values[i - 1] > threshold > values[i]:
            crossings += 1

    return crossings


def make_same_length(times, list_of_values):
    diff = len(times) - len(list_of_values)
    if diff > 0:
        times = times[:-diff]
    elif diff < 0:
        list_of_values = list_of_values[:diff]

    return times, list_of_values


class TestPlot:

    def __init__(self,
                 test_state,
                 map_points,
                 trajectory_points,
                 vehicle_waypoints,
                 final_steering_angle_list,
                 lane_deviation_list,
                 max_speed,
                 test_tele_operator_steering_angle_list, test_dsc_steering_angle_list,
                 test_heading_steering_angle_list, test_lane_deviation_steering_angle_list,
                 heading_error_list, test_speed_list, test_time, lateral_acceleration_list):

        total_steering_angle = None
        steering_angle_threshold_crossings = None
        total_lane_deviation = None
        average_lane_deviation = None
        median_lane_deviation = None
        max_lane_deviation = None

        # TEST STATE
        speed = int(round(test_state[0], 1))
        controller_state = test_state[1]
        delay = test_state[2]

        # HEADING ERROR
        heading_error_list = [abs(heading_error) for heading_error in heading_error_list]
        median_heading_error = round(abs(np.median(heading_error_list)), 3)
        average_heading_error = round(sum(heading_error_list) / len(heading_error_list), 3)
        max_heading_error = round(max(heading_error_list), 3)
        summed_heading_error = round(sum(heading_error_list), 3)

        # STEERING ANGLES
        tele_operator_steering_amounts = [abs(steering_angle) for steering_angle in
                                          test_tele_operator_steering_angle_list]
        average_tele_operator_steering_angle = round(sum(tele_operator_steering_amounts) /
                                                     len(tele_operator_steering_amounts), 3)
        median_tele_operator_steering_angle = round(abs(np.median(tele_operator_steering_amounts)), 3)
        max_tele_operator_steering_angle = round(max(tele_operator_steering_amounts), 3)

        # LATERAL ACCELERATION
        lateral_acceleration_amounts = [abs(lateral_acceleration) for lateral_acceleration in lateral_acceleration_list]
        average_lateral_acceleration = round(sum(lateral_acceleration_amounts) / len(lateral_acceleration_amounts), 3)
        median_lateral_acceleration = round(abs(np.median(lateral_acceleration_amounts)), 3)
        max_lateral_acceleration = round(max(lateral_acceleration_amounts), 3)
        summed_lateral_acceleration = round(sum(lateral_acceleration_amounts), 3)

        # SPEED
        average_speed = round(sum(test_speed_list) / len(test_speed_list), 3)

        if controller_state:
            controller_state = "ON"
        else:
            controller_state = "OFF"

        test_title = f"{controller_state}_{delay}ms_{speed}mps"

        self.plot_directory = self.create_directory(test_title)
        self.pickle_directory = f"{self.plot_directory}/Pickle_files"
        os.mkdir(self.pickle_directory)

        if vehicle_waypoints:
            self.plot_map(map_points, trajectory_points, vehicle_waypoints, delay)

        if final_steering_angle_list:
            total_steering_angle, steering_angle_threshold_crossings = \
                self.steering_plot(final_steering_angle_list, speed, delay, test_time)
            total_steering_angle = round(total_steering_angle, 3)

        if lane_deviation_list:
            total_lane_deviation, average_lane_deviation, median_lane_deviation = (
                self.lane_deviation_plot(lane_deviation_list, speed, delay, test_time))
            total_lane_deviation = round(total_lane_deviation, 3)
            average_lane_deviation = round(average_lane_deviation, 3)
            median_lane_deviation = round(median_lane_deviation, 3)
            max_lane_deviation = round(max(lane_deviation_list), 3)

        if lateral_acceleration_list:
            self.lateral_acceleration_plot(lateral_acceleration_list, speed, delay, test_time)

        with open(f"{self.pickle_directory}/speed.pkl", 'wb') as file:
            pickle.dump((test_speed_list, test_time), file)

        with open(f"{self.pickle_directory}/heading_error.pkl", 'wb') as file:
            pickle.dump((heading_error_list, test_time), file)

        with open(f"{self.pickle_directory}/dsc_steering.pkl", 'wb') as file:
            pickle.dump((test_dsc_steering_angle_list, test_time), file)

        with open(f"{self.pickle_directory}/heading_steering.pkl", 'wb') as file:
            pickle.dump((test_heading_steering_angle_list, test_time), file)

        with open(f"{self.pickle_directory}/lane_deviation_steering.pkl", 'wb') as file:
            pickle.dump((test_lane_deviation_steering_angle_list, test_time), file)

        with open(f"{self.pickle_directory}/lateral_acceleration.pkl", 'wb') as file:
            pickle.dump((lateral_acceleration_list, test_time), file)

        # Save test result
        test_result = f"{self.plot_directory}/test_result.txt"
        with open(test_result, "w") as file:
            file.write(f"TEST STATE:\n")
            file.write(f"Controller state: {controller_state}\n")
            file.write(f"Speed: {speed} m/s\n")
            file.write(f"Delay: {delay} ms\n")
            file.write("--------------------\n")
            file.write("TIME AND SPEED:\n")
            file.write(f"test time: {round(test_time, 3)} s\n")
            file.write(f"Max speed: {round(max_speed, 3)} m/s\n")
            file.write(f"Average speed: {average_speed} m/s\n")
            file.write("--------------------\n")
            file.write("STEERING ANGLES:\n")
            file.write(f"Summed tele-operator steering angle: {total_steering_angle} rad\n")
            file.write(f"Average tele-operator steering angle: {average_tele_operator_steering_angle} rad\n")
            file.write(f"Median tele-operator steering angle: {median_tele_operator_steering_angle} rad\n")
            file.write(f"Max tele-operator steering angle: {max_tele_operator_steering_angle} rad\n")
            file.write(f"Steering angle threshold crossings: {steering_angle_threshold_crossings} rad\n")
            file.write("--------------------\n")
            file.write("LANE DEVIATION:\n")
            file.write(f"Max lane deviation: {max_lane_deviation} m\n")
            file.write(f"Summed lane deviation: {total_lane_deviation} m\n")
            file.write(f"Average lane deviation: {average_lane_deviation} m\n")
            file.write(f"Median lane deviation: {median_lane_deviation} m\n")
            file.write("--------------------\n")
            file.write("HEADING ERROR:\n")
            file.write(f"Max heading error: {max_heading_error} rad\n")
            file.write(f"Summed heading error: {summed_heading_error} rad\n")
            file.write(f"Average heading error: {average_heading_error} rad\n")
            file.write(f"Median heading error: {median_heading_error} rad\n")
            file.write("--------------------\n")
            file.write("LATERAL ACCELERATION:\n")
            file.write(f"Max lateral acceleration: {max_lateral_acceleration} m/s^2\n")
            file.write(f"Summed lateral acceleration: {summed_lateral_acceleration} m/s^2\n")
            file.write(f"Average lateral acceleration: {average_lateral_acceleration} m/s^2\n")
            file.write(f"Median lateral acceleration: {median_lateral_acceleration} m/s^2\n")

    @staticmethod
    def create_directory(test_title):
        test_plot_directory = "src/control_command_corrector/src/Test_plots"

        number_of_tests = len([f.path for f in os.scandir(test_plot_directory) if f.is_dir()])

        directory_path = f"{test_plot_directory}/T{number_of_tests + 1}_{test_title}"
        os.mkdir(directory_path)
        return directory_path

    def plot_map(self, map_points, trajectory_points, vehicle_points, delay):
        x_map_coordinates = []
        y_map_coordinates = []
        for point in map_points:
            x_map_coordinates.append(point[0])
            y_map_coordinates.append(point[1])

        x_coordinates = []
        y_coordinates = []
        for point in trajectory_points:
            x_coordinates.append(point[0])
            y_coordinates.append(point[1])

        x_vehicle_coordinates = []
        y_vehicle_coordinates = []
        for point in vehicle_points:
            x_vehicle_coordinates.append(point[0])
            y_vehicle_coordinates.append(point[1])

        plt.title(f'delay: {delay}')
        plt.axis("equal")
        plt.scatter(x_coordinates, y_coordinates, s=0.1, color="green")
        plt.scatter(x_map_coordinates, y_map_coordinates, s=0.1, color="grey")
        plt.scatter(x_vehicle_coordinates, y_vehicle_coordinates, s=0.1, color="blue")

        plt.gca().invert_yaxis()

        plt_location = f"{self.plot_directory}/map_plot"
        plt.savefig(plt_location)
        plt.close()

        with open(f"{self.pickle_directory}/map.pkl", 'wb') as file:
            pickle.dump((map_points, trajectory_points, vehicle_points, delay), file)

    def steering_plot(self, steering_angle_list, speed, delay, test_time):

        times = np.arange(0, test_time, test_time / len(steering_angle_list))

        tele_operator_steering_angle_list = [angle_tuple[0] for angle_tuple in steering_angle_list]
        controller_steering_angle_list = [angle_tuple[1] for angle_tuple in steering_angle_list]

        total_tele_operator_steering_angle = 0
        for steering_angle in tele_operator_steering_angle_list:
            total_tele_operator_steering_angle = total_tele_operator_steering_angle + abs(steering_angle)

        steering_threshold = 0.05
        steering_angle_threshold_crossings = count_threshold_crossings(tele_operator_steering_angle_list,
                                                                       steering_threshold)
        if len(times) != len(tele_operator_steering_angle_list):
            times, tele_operator_steering_angle_list = make_same_length(times, tele_operator_steering_angle_list)

        if len(times) != len(controller_steering_angle_list):
            times, controller_steering_angle_list = make_same_length(times, controller_steering_angle_list)

        plt.plot(times, tele_operator_steering_angle_list, linestyle='-', label="Tele-operator")
        plt.plot(times, controller_steering_angle_list, linestyle='-', label="Controller")

        plt.title(
            f'Speed: {speed}, delay: {delay}ms, total steering angle: {round(total_tele_operator_steering_angle, 3)},'
            f' crossings: {round(steering_angle_threshold_crossings, 3)}')
        plt.xlabel('Time (seconds)')
        plt.ylabel('Values')
        plt.axis('equal')
        plt.legend()
        plt.grid(True)

        plt_location = f"{self.plot_directory}/steering_plot"
        plt.savefig(plt_location)
        plt.close()

        with open(f"{self.pickle_directory}/steering.pkl", 'wb') as file:
            pickle.dump((steering_angle_list, speed, delay, test_time), file)

        return total_tele_operator_steering_angle, steering_angle_threshold_crossings

    def lane_deviation_plot(self, lane_deviation_list, speed, delay, test_time):
        total_lane_deviation = 0
        for lane_deviation in lane_deviation_list:
            total_lane_deviation = total_lane_deviation + abs(lane_deviation)

        times = np.arange(0, test_time, test_time / len(lane_deviation_list))

        average_lane_deviation = total_lane_deviation / len(lane_deviation_list)
        median_lane_deviation = abs(np.median(lane_deviation_list))

        if len(times) != len(lane_deviation_list):
            times, lane_deviation_list = make_same_length(times, lane_deviation_list)

        plt.plot(times, lane_deviation_list, linestyle='-', label="lane deviation")
        plt.title(f'Speed: {speed},'
                  f' delay: {delay}ms,'
                  f' avg lane dev: {round(average_lane_deviation, 3)}'
                  f' total lane dev: {round(total_lane_deviation, 3)}, '
                  f'median lane dev: {round(median_lane_deviation, 3)}')
        plt.xlabel('Time (seconds)')
        plt.ylabel('lane deviation (meters)')
        plt.axis('equal')
        plt.legend()
        plt.grid(True)

        plt_location = f"{self.plot_directory}/lane_deviation_plot"
        plt.savefig(plt_location)
        plt.close()

        with open(f"{self.pickle_directory}/lane_deviation.pkl", 'wb') as file:
            pickle.dump((lane_deviation_list, speed, delay, test_time), file)

        return total_lane_deviation, average_lane_deviation, median_lane_deviation

    def lateral_acceleration_plot(self, lateral_acceleration_list, speed, delay, test_time):

        times = np.arange(0, test_time, test_time / len(lateral_acceleration_list))

        if len(times) != len(lateral_acceleration_list):
            times, tele_operator_steering_angle_list = make_same_length(times, lateral_acceleration_list)

        plt.plot(times, lateral_acceleration_list, linestyle='-', label="lateral acceleration")

        plt.title(
            f'Speed: {speed}, delay: {delay}ms')
        plt.xlabel('Time (seconds)')
        plt.ylabel('lateral acceleration (m/s^2)')
        plt.axis('equal')
        plt.legend()
        plt.grid(True)

        plt_location = f"{self.plot_directory}/lateral_acceleration_plot"
        plt.savefig(plt_location)
        plt.close()
