import os
import pickle
import numpy as np


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


def calculate_median(directory):
    with open(f"{directory}/Pickle_files/lane_deviation.pkl", 'rb') as file:
        (lane_deviation_list, speed, delay, test_time) = pickle.load(file)

    abs_lane_deviation_list = [abs(lane_deviation) for lane_deviation in lane_deviation_list]
    median_lane_deviation = np.median(abs_lane_deviation_list)
    return median_lane_deviation


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


        median = calculate_median(test_directory)
        test_result_file_path = f"{test_directory}/test_result.txt"
        update_median(test_result_file_path, median)

        return True


def main():
    test_result_directory = r"/home/danny/Documents/University/Bachelorarbeit/Test_Results"
    user_directories = [f.path for f in os.scandir(test_result_directory) if f.is_dir()]

    for user_dir in user_directories:
        user = user_dir.split("/")[-1]
        if user == "Philipp":
            UpdateMedians(user_dir)

if __name__ == "__main__":
    main()