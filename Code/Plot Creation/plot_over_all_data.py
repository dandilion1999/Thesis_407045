import os
import pickle
import numpy as np
from plotting import create_error_box_plots


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


def get_lane_deviation_list(directory):
    with open(f"{directory}/Pickle_files/lane_deviation.pkl", 'rb') as file:
        (lane_deviation_list, speed, delay, test_time) = pickle.load(file)

    abs_lane_deviation_list = [abs(lane_deviation) for lane_deviation in lane_deviation_list]
    # median_lane_deviation = np.median(abs_lane_deviation_list)

    under_3_list = [lane_deviation for lane_deviation in abs_lane_deviation_list if lane_deviation < 3]

    return under_3_list


def get_teleop_steering_inputs(directory):
    with open(f"{directory}/Pickle_files/steering.pkl", 'rb') as file:
        (steering_angle_list, speed, delay, test_time) = pickle.load(file)

    tele_operator_steering_angle_list = [angle_tuple[0] for angle_tuple in steering_angle_list]
    abs_teleop_steering_angle_list = [abs(steering_input) for steering_input in tele_operator_steering_angle_list]

    return abs_teleop_steering_angle_list

def get_test_directory(test_user_directory, test_key):
    test_run_directories = [f.path for f in os.scandir(test_user_directory) if f.is_dir()]

    for directory in test_run_directories:
        directory_name = directory.split("\\")[-1]

        if test_key in directory_name:
            return directory


class UserData:

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

        lane_deviation_list = get_lane_deviation_list(test_directory)
        # abs_teleop_inputs = get_teleop_steering_inputs(test_directory)
        return lane_deviation_list


def main():
    test_result_directory = r"/home/danny/Documents/University/Bachelorarbeit/Test_Results"
    user_directories = [f.path for f in os.scandir(test_result_directory) if f.is_dir()]

    userdata_object_list = []
    for user_dir in user_directories:
        user = user_dir.split("/")[-1]
        print(user)
        if user != "Hichem":
            userdata_object_list.append(UserData(user_dir))

    all_baseline_values = []

    all_two_off_values = []
    all_two_on_values = []

    all_three_off_values = []
    all_three_on_values = []

    all_four_off_values = []
    all_four_on_values = []

    for user in userdata_object_list:
        all_baseline_values.extend(user.baseline)
        all_two_off_values.extend(user.test_two_hundred_off)
        all_two_on_values.extend(user.test_two_hundred_on)
        all_three_off_values.extend(user.test_three_hundred_off)
        all_three_on_values.extend(user.test_three_hundred_on)
        all_four_off_values.extend(user.test_four_hundred_off)
        all_four_on_values.extend(user.test_four_hundred_on)


    data = [all_baseline_values,
            all_two_off_values, all_two_on_values,
            all_three_off_values, all_three_on_values,
            all_four_off_values, all_four_on_values]

    off_mean = np.mean(all_two_off_values)
    on_mean = np.mean(all_two_on_values)
    print(off_mean)
    print(on_mean)
    percent_reduction = ((off_mean - on_mean) / off_mean) * 100
    print("two", percent_reduction)

    off_mean = np.mean(all_three_off_values)
    on_mean = np.mean(all_three_on_values)
    print(off_mean)
    print(on_mean)
    percent_reduction = ((off_mean - on_mean) / off_mean) * 100
    print("three", percent_reduction)

    off_mean = np.mean(all_four_off_values)
    on_mean = np.mean(all_four_on_values)
    print(off_mean)
    print(on_mean)
    percent_reduction = ((off_mean - on_mean) / off_mean) * 100
    print("four", percent_reduction)


    create_error_box_plots(data)


if __name__ == "__main__":
    main()