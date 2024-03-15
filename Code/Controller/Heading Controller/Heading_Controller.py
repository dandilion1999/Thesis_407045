from numpy import linspace
from pyit2fls import T1FS, T1Mamdani, rtri_mf, ltri_mf, tri_mf, T1FS_plot, gaussian_mf, singleton_mf
import numpy as np  # add dependencies in cmake/package.xml
import matplotlib.pyplot as plt


class HeadingController:

    def __init__(self):
        """
        CONTROLLER SETUP
        """
        """INPUT AND OUTPUT DOMAINS"""
        self.heading_error_domain = linspace(-0.3, 0.3, 100)
        steering_angle_domain = linspace(-1, 1, 100)

        """DEFINE INPUTS AND OUTPUT"""
        self.TRJ_CTRL = T1Mamdani(engine="Minimum", defuzzification="CoG")
        self.TRJ_CTRL.add_input_variable("heading_error")

        self.TRJ_CTRL.add_output_variable("steering_angle")

        """HEADING ERROR"""
        heading_error_left = T1FS(self.heading_error_domain, rtri_mf, [0.0, -0.05, 1])
        no_heading_error = T1FS(self.heading_error_domain, tri_mf, [-0.01, 0, 0.01, 1])
        heading_error_right = T1FS(self.heading_error_domain, ltri_mf, [0.0, 0.05, 1])
        # T1FS_plot(heading_error_left, no_heading_error, heading_error_right, legends=("left", "no", "right"))

        """STEERING ANGLE"""
        steering_angle_left = T1FS(steering_angle_domain, tri_mf, [-0.25, -0.15, 0.0, 1])
        steering_angle_straight = T1FS(steering_angle_domain, tri_mf, [-0.05, 0, 0.05, 1])
        steering_angle_right = T1FS(steering_angle_domain, tri_mf, [0.0, 0.15, 0.25, 1])

        # T1FS_plot(steering_angle_left, steering_angle_straight,
        #         steering_angle_right,
        #     legends=("steering_angle_hard_left", "left", "straight", "right", "steering_angle_hard_right"))

        """FUZZY RULES"""

        self.TRJ_CTRL.add_rule([("heading_error", heading_error_left)], [("steering_angle", steering_angle_right)])
        self.TRJ_CTRL.add_rule([("heading_error", no_heading_error)], [("steering_angle", steering_angle_straight)])
        self.TRJ_CTRL.add_rule([("heading_error", heading_error_right)], [("steering_angle", steering_angle_left)])

        self.create_plot()

    def evaluate(self, heading_error):

        something, output_dict = self.TRJ_CTRL.evaluate({
            "heading_error": heading_error})

        steering_angle = output_dict['steering_angle']

        return steering_angle

    def create_plot(self):

        steering_angle_outputs = []

        for heading_error in self.heading_error_domain:
            steering_angle_outputs.append(self.evaluate(heading_error))

        plt.plot(self.heading_error_domain, steering_angle_outputs)

        plt.xlabel("heading error (degree)")
        plt.ylabel("steering input")
        plt.grid()
        plt.show()


if __name__ == "__main__":
    try:
        HeadingController()
    except Exception as exp:
        print(exp)
