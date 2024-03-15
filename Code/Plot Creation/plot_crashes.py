import os
import pickle
import numpy as np
import matplotlib.pyplot as plt



def make_same_length(times, list_of_values):
    diff = len(times) - len(list_of_values)
    if diff > 0:
        times = times[:-diff]
    elif diff < 0:
        list_of_values = list_of_values[:diff]

    return times, list_of_values


def update_median(test_result_file_path, median):

    with open(test_result_file_path, 'r') as file:
        lines = file.readlines()

    median_line_number = 21
    median = round(median, 3)
    # Replace the content at the specified line
    if 0 <= median_line_number < len(lines):
        lines[median_line_number] = f"Median lane deviation: {median} m" + '\n'
    else:
        print(f"Line number {median_line_number + 1} is out of range. No changes were made.")

    # Write the modified contents back to the file
    with open(test_result_file_path, 'w') as file:
        file.writelines(lines)


def get_from_pickle(directory, pickle_file_name):
    with open(f"{directory}/Pickle_files/{pickle_file_name}.pkl", 'rb') as file:
        (test_speed_list, test_time) = pickle.load(file)

    return test_speed_list, test_time


def get_vehicle_points_from_pickle(directory):

    with open(f"{directory}/Pickle_files/map.pkl", 'rb') as file:
        (map_points, trajectory_points, vehicle_points, delay) = pickle.load(file)
    return map_points, trajectory_points, vehicle_points

def get_test_directory(test_user_directory, test_key):
    test_run_directories = [f.path for f in os.scandir(test_user_directory) if f.is_dir()]

    for directory in test_run_directories:
        directory_name = directory.split("\\")[-1]

        if test_key in directory_name:
            return directory


class UpdateMedians:

    def __init__(self, test_user_directory):
        self.user = test_user_directory.split("/")[-1]
        self.test_user_directory = test_user_directory

        self.baseline = self.get_test_run_data("OFF_0ms")
        if not self.baseline:
            self.baseline = self.get_test_run_data("ON_0ms")

        self.test_two_hundred_off = self.get_test_run_data("OFF_200ms")
        self.test_two_hundred_on = self.get_test_run_data("ON_200ms")

        self.test_three_hundred_off = self.get_test_run_data("OFF_300ms")
        self.test_three_hundred_on = self.get_test_run_data("ON_300ms")

        self.test_four_hundred_off = self.get_test_run_data("OFF_400ms")
        self.test_four_hundred_on = self.get_test_run_data("ON_400ms")

    def get_test_run_data(self, test_key):

        test_directory = get_test_directory(self.test_user_directory, test_key)

        if not test_directory:
            print(f"couldn't find {self.test_user_directory}, {test_key}")
            return

        test_attribute = "speed"
        speed_data, test_time = get_from_pickle(test_directory, test_attribute)
        times = np.arange(0, test_time, test_time / len(speed_data))

        map_points, trajectory_points, vehicle_points = get_vehicle_points_from_pickle(test_directory)
        crash_times = []
        crash_found = False

        for i, speed in enumerate(speed_data):
            if speed < 1 and not crash_found:
                crash_times.append(times[i])
                crash_found = True

            if speed > 5:
                crash_found = False

        vehicle_times = np.arange(0, test_time, test_time / len(vehicle_points))
        crash_locations = []

        for crash_time in crash_times:
            time_index = None
            smallest_difference = 99
            for i, time in enumerate(vehicle_times):
                difference = abs(crash_time - time)
                if difference < smallest_difference:
                    smallest_difference = difference
                    time_index = i

            crash_location = vehicle_points[time_index]
            crash_locations.append(crash_location)

        return crash_locations


def plot_crash_locations(map_points, trajectory_points,  all_two_off_crashes, all_three_off_crashes, all_four_off_crashes):

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

    x_two_off = []
    y_two_off = []
    for point in all_two_off_crashes:
        x_two_off.append(point[0])
        y_two_off.append(point[1])

    x_three_off = []
    y_three_off = []
    for point in all_three_off_crashes:
        x_three_off.append(point[0])
        y_three_off.append(point[1])

    x_four_off = []
    y_four_off = []
    for point in all_four_off_crashes:
        x_four_off.append(point[0])
        y_four_off.append(point[1])

    plt.axis("equal")
    plt.scatter(x_map_coordinates, y_map_coordinates, s=0.05, color="grey")
    plt.scatter(x_coordinates, y_coordinates, s=0.2, color="green")
    plt.scatter(x_four_off, y_four_off, s=12, color="blue", marker=",", label="400 ms")
    plt.scatter(x_three_off, y_three_off, s=12, color="orange", marker=",", label="300 ms")

    plt.legend()


    plt.gca().invert_yaxis()
    plt.show()


def get_map_and_trajectory_points(test_user_directory, test_key):

    test_directory = get_test_directory(test_user_directory, test_key)

    if not test_directory:
        print(f"couldn't find {test_user_directory}, {test_key}")
        return

    map_points, trajectory_points, vehicle_points = get_vehicle_points_from_pickle(test_directory)
    return map_points, trajectory_points


def main():

    test_result_directory = r"/home/danny/Documents/University/Bachelorarbeit/Test_Results"
    user_directories = [f.path for f in os.scandir(test_result_directory) if f.is_dir()]

    user_object_list = []
    for user_dir in user_directories:
        if user_dir.split("/")[-1] != "Hichem":
            user_object_list.append(UpdateMedians(user_dir))

    all_baseline_crashes = []

    all_two_off_crashes = []
    all_two_on_crashes = []

    all_three_off_crashes = []
    all_three_on_crashes = []

    all_four_off_crashes = []
    all_four_on_crashes = []

    for user in user_object_list:
        if user.baseline:
            all_baseline_crashes.extend(user.baseline)

        if user.test_two_hundred_off:
            all_two_off_crashes.extend(user.test_two_hundred_off)

        if user.test_two_hundred_on:
            all_two_on_crashes.extend(user.test_two_hundred_on)

        if user.test_three_hundred_off:
            all_three_off_crashes.extend(user.test_three_hundred_off)

        if user.test_three_hundred_on:
            all_three_on_crashes.extend(user.test_three_hundred_on)

        if user.test_four_hundred_off:
            all_four_off_crashes.extend(user.test_four_hundred_off)

        if user.test_four_hundred_on:
            all_four_on_crashes.extend(user.test_four_hundred_on)

    map_points, trajectory_points = get_map_and_trajectory_points(user_directories[0], "OFF_0ms")

    print(len(all_two_off_crashes))
    print(len(all_three_off_crashes))
    print(len(all_four_off_crashes))
    print(len(all_two_on_crashes))
    print(len(all_three_on_crashes))
    print(len(all_four_on_crashes))

    plot_crash_locations(map_points, trajectory_points, all_two_off_crashes, all_three_off_crashes, all_four_off_crashes)





if __name__ == "__main__":
    main()
