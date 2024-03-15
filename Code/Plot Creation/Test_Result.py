import os
import re

import numpy as np

from plotting import create_error_box_plots, create_side_by_side_error_box_plots


def extract_number(line):
    # Use regular expression to find numbers in the line
    match = re.search(r'(\d+(\.\d+)?)\s+\w+', line)
    if match:
        # Convert the matched string to a float and return
        return float(match.group(1))
    else:
        # If no number is found, return None or raise an error
        return None


def get_data_from_result_file(test_result_file_path):
    with open(test_result_file_path) as result_file:
        result_lines = result_file.readlines()

    controller_state = extract_number(result_lines[1])
    speed = extract_number(result_lines[2])
    delay = extract_number(result_lines[3])

    test_time = extract_number(result_lines[6])
    max_speed = extract_number(result_lines[7])
    average_speed = extract_number(result_lines[8])

    summed_teleoperator_steering_angles = extract_number(result_lines[11])
    average_teleoperator_steering_angle = extract_number(result_lines[12])
    median_teleoperator_steering_angle = extract_number(result_lines[13])
    max_tele_operator_steering_angle = extract_number(result_lines[14])
    steering_angle_threshold = extract_number(result_lines[15])

    max_lane_dev = extract_number(result_lines[18])
    summed_lane_dev = extract_number(result_lines[19])
    average_lane_dev = extract_number(result_lines[20])
    median_lane_dev = extract_number(result_lines[21])

    max_heading_error = extract_number(result_lines[24])
    summed_heading_error = extract_number(result_lines[25])
    average_heading_error = extract_number(result_lines[26])
    median_heading_error = extract_number(result_lines[27])

    max_lateral_acceleration = extract_number(result_lines[30])
    summed_lateral_acceleration = extract_number(result_lines[31])
    average_lateral_acceleration = extract_number(result_lines[32])
    median_lateral_acceleration = extract_number(result_lines[33])

    return controller_state, speed, delay, test_time, max_speed, average_speed, \
        summed_teleoperator_steering_angles, average_teleoperator_steering_angle, median_teleoperator_steering_angle, \
        max_tele_operator_steering_angle, steering_angle_threshold, \
        max_lane_dev, summed_lane_dev, average_lane_dev, median_lane_dev, \
        max_heading_error, summed_heading_error, average_heading_error, median_heading_error, \
        max_lateral_acceleration, summed_lateral_acceleration, average_lateral_acceleration, median_lateral_acceleration


def get_test_result_file(test_user_directory, test_key):
    test_run_directories = [f.path for f in os.scandir(test_user_directory) if f.is_dir()]

    for directory in test_run_directories:
        directory_name = directory.split("/")[-1]

        if test_key in directory_name:
            return f"{directory}/test_result.txt"


class TestResult:

    def __init__(self, test_user_directory):
        self.user = test_user_directory.split("\\")[-1]
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

        test_result_file_path = get_test_result_file(self.test_user_directory, test_key)
        if not test_result_file_path:
            print(f"couldn't find {self.test_user_directory}, {test_key}")
            return

        get_data_from_result_file(test_result_file_path)

        (controller_state, speed, delay, test_time, max_speed, average_speed,
         summed_teleoperator_steering_angles, average_teleoperator_steering_angle, median_teleoperator_steering_angle,
         max_tele_operator_steering_angle, steering_angle_threshold,
         max_lane_dev, summed_lane_dev, average_lane_dev, median_lane_dev,
         max_heading_error, summed_heading_error, average_heading_error, median_heading_error,
         max_lateral_acceleration, summed_lateral_acceleration, average_lateral_acceleration,
         median_lateral_acceleration) = get_data_from_result_file(test_result_file_path)

        test_run_data_object = TestRunData(controller_state, speed, delay, test_time, max_speed, average_speed,
                                           summed_teleoperator_steering_angles, average_teleoperator_steering_angle,
                                           median_teleoperator_steering_angle,
                                           max_tele_operator_steering_angle, steering_angle_threshold,
                                           max_lane_dev, summed_lane_dev, average_lane_dev, median_lane_dev,
                                           max_heading_error, summed_heading_error, average_heading_error,
                                           median_heading_error,
                                           max_lateral_acceleration, summed_lateral_acceleration,
                                           average_lateral_acceleration, median_lateral_acceleration)

        return test_run_data_object


class TestRunData:

    def __init__(self, controller_state, speed, delay,
                 test_time, max_speed, average_speed,
                 summed_teleoperator_steering_angles, average_teleoperator_steering_angle,
                 median_teleoperator_steering_angle,
                 max_tele_operator_steering_angle, steering_angle_threshold,
                 max_lane_dev, summed_lane_dev, average_lane_dev, median_lane_dev,
                 max_heading_error, summed_heading_error, average_heading_error, median_heading_error,
                 max_lateral_acceleration, summed_lateral_acceleration, average_lateral_acceleration,
                 median_lateral_acceleration):
        self.median_lateral_acceleration = median_lateral_acceleration
        self.average_lateral_acceleration = average_lateral_acceleration
        self.summed_lateral_acceleration = summed_lateral_acceleration
        self.max_lateral_acceleration = max_lateral_acceleration
        self.median_heading_error = median_heading_error
        self.average_heading_error = average_heading_error
        self.summed_heading_error = summed_heading_error
        self.max_heading_error = max_heading_error
        self.median_lane_dev = median_lane_dev
        self.average_lane_dev = average_lane_dev
        self.summed_lane_dev = summed_lane_dev
        self.max_lane_dev = max_lane_dev
        self.steering_angle_threshold = steering_angle_threshold
        self.max_tele_operator_steering_angle = max_tele_operator_steering_angle
        self.median_teleoperator_steering_angle = median_teleoperator_steering_angle
        self.average_teleoperator_steering_angle = average_teleoperator_steering_angle
        self.summed_teleoperator_steering_angles = summed_teleoperator_steering_angles
        self.average_speed = average_speed
        self.max_speed = max_speed
        self.test_time = test_time
        self.delay = delay
        self.speed = speed
        self.controller_state = controller_state


def get_data_of_attibute(test_results, test_attribute):
    baseline_values = [getattr(test_user.baseline, test_attribute) for test_user in test_results]
    baseline_std_dev = np.std(baseline_values)
    baseline_mean = np.mean(baseline_values)

    two_hundred_off_values = [getattr(test_user.test_two_hundred_off, test_attribute) for test_user in test_results]
    two_hundred_off_std_dev = np.std(two_hundred_off_values)
    two_hundred_off_mean = np.mean(two_hundred_off_values)

    two_hundred_on_values = [getattr(test_user.test_two_hundred_on, test_attribute) for test_user in test_results]
    two_hundred_on_std_dev = np.std(two_hundred_on_values)
    two_hundred_on_mean = np.mean(two_hundred_on_values)

    three_hundred_off_values = [getattr(test_user.test_three_hundred_off, test_attribute) for test_user in test_results]
    three_hundred_off_std_dev = np.std(three_hundred_off_values)
    three_hundred_off_mean = np.mean(three_hundred_off_values)

    three_hundred_on_values = [getattr(test_user.test_three_hundred_on, test_attribute) for test_user in test_results]
    three_hundred_on_std_dev = np.std(three_hundred_on_values)
    three_hundred_on_mean = np.mean(three_hundred_on_values)

    four_hundred_off_values = [getattr(test_user.test_four_hundred_off, test_attribute) for test_user in test_results]
    four_hundred_off_std_dev = np.std(four_hundred_off_values)
    four_hundred_off_mean = np.mean(four_hundred_off_values)

    four_hundred_on_values = [getattr(test_user.test_four_hundred_on, test_attribute) for test_user in test_results]
    four_hundred_on_std_dev = np.std(four_hundred_on_values)
    four_hundred_on_mean = np.mean(four_hundred_on_values)

    create_error_box_plots([baseline_values,
                            two_hundred_off_values, two_hundred_on_values,
                            three_hundred_off_values, three_hundred_on_values,
                            four_hundred_off_values, four_hundred_on_values], test_attribute)

    categrories = ["Baseline", "200_off", "200_on", "300_off", "300_on", "400_off", "400_on"]
    means = [baseline_mean,
             two_hundred_off_mean, two_hundred_on_mean,
             three_hundred_off_mean, three_hundred_on_mean,
             four_hundred_off_mean, four_hundred_on_mean]

    std_devs = [baseline_std_dev,
                two_hundred_off_std_dev, two_hundred_on_std_dev,
                three_hundred_off_std_dev, three_hundred_on_std_dev,
                four_hundred_off_std_dev, four_hundred_on_std_dev]

    return categrories, means, std_devs


def get_data_of_attibute_for_side_by_side(test_results, test_attribute, test_attribute_2):
    baseline_values = [getattr(test_user.baseline, test_attribute) for test_user in test_results]

    two_hundred_off_values = [getattr(test_user.test_two_hundred_off, test_attribute) for test_user in test_results]
    two_hundred_on_values = [getattr(test_user.test_two_hundred_on, test_attribute) for test_user in test_results]

    three_hundred_off_values = [getattr(test_user.test_three_hundred_off, test_attribute) for test_user in test_results]
    three_hundred_on_values = [getattr(test_user.test_three_hundred_on, test_attribute) for test_user in test_results]

    four_hundred_off_values = [getattr(test_user.test_four_hundred_off, test_attribute) for test_user in test_results]
    four_hundred_on_values = [getattr(test_user.test_four_hundred_on, test_attribute) for test_user in test_results]

    baseline_values_2 = [getattr(test_user.baseline, test_attribute_2) for test_user in test_results]

    two_hundred_off_values_2 = [getattr(test_user.test_two_hundred_off, test_attribute_2) for test_user in test_results]
    two_hundred_on_values_2 = [getattr(test_user.test_two_hundred_on, test_attribute_2) for test_user in test_results]

    three_hundred_off_values_2 = [getattr(test_user.test_three_hundred_off, test_attribute_2) for test_user in
                                  test_results]
    three_hundred_on_values_2 = [getattr(test_user.test_three_hundred_on, test_attribute_2) for test_user in test_results]

    four_hundred_off_values_2 = [getattr(test_user.test_four_hundred_off, test_attribute_2) for test_user in test_results]
    four_hundred_on_values_2 = [getattr(test_user.test_four_hundred_on, test_attribute_2) for test_user in test_results]

    data_1 = [baseline_values,
              two_hundred_off_values, two_hundred_on_values,
              three_hundred_off_values, three_hundred_on_values,
              four_hundred_off_values, four_hundred_on_values]

    data_2 = [baseline_values_2,
              two_hundred_off_values_2, two_hundred_on_values_2,
              three_hundred_off_values_2, three_hundred_on_values_2,
              four_hundred_off_values_2, four_hundred_on_values_2]

    create_side_by_side_error_box_plots(data_1, data_2)


def main():
    test_result_directory = r"/home/danny/Documents/University/Bachelorarbeit/Test_Results"
    user_directories = [f.path for f in os.scandir(test_result_directory) if f.is_dir()]

    test_results = []
    for user_dir in user_directories:
        if user_dir.split("/")[-1] != "Hichem":
            # print(user_dir.split("/")[-1])
            test_results.append(TestResult(user_dir))

    # ALL USERS TOGETHER
    # max lateral acceleration is unusually high

    test_attribute = "steering_angle_threshold"
    # categories, means, std_devs = get_data_of_attibute(test_results, test_attribute)
    # create_plot(test_attribute, categories, means, std_devs)

    # get_data_of_attibute_for_side_by_side(test_results, "steering_angle_threshold", "median_teleoperator_steering_angle")

    all_off = []
    all_on = []
    # EACH USER SEPARATELY
    for test_result in test_results:
        baseline_value = getattr(test_result.baseline, test_attribute)

        two_hundred_off_value = getattr(test_result.test_two_hundred_off, test_attribute)
        two_hundred_on_value = getattr(test_result.test_two_hundred_on, test_attribute)

        three_hundred_off_value = getattr(test_result.test_three_hundred_off, test_attribute)
        three_hundred_on_value = getattr(test_result.test_three_hundred_on, test_attribute)

        four_hundred_off_value = getattr(test_result.test_four_hundred_off, test_attribute)
        four_hundred_on_value = getattr(test_result.test_four_hundred_on, test_attribute)

        data = [baseline_value,
                two_hundred_off_value, two_hundred_on_value,
                three_hundred_off_value, three_hundred_on_value,
                four_hundred_off_value, four_hundred_on_value]
        off_data = [two_hundred_off_value, three_hundred_off_value, four_hundred_off_value]
        on_data = [two_hundred_on_value, three_hundred_on_value, four_hundred_on_value]

        all_off.extend(off_data)
        all_on.extend(on_data)


        # create_user_plot(data, test_attribute, test_result.user)
    off_mean = np.mean(all_off)
    on_mean = np.mean(all_on)
    print("off mean: ", off_mean)
    print("on mean: ", on_mean)

    percent_reduction = ((off_mean - on_mean) / off_mean) * 100
    print(percent_reduction)



if __name__ == "__main__":
    main()
