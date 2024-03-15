from numpy import linspace
from pyit2fls import T1FS, T1Mamdani, rtri_mf, ltri_mf, tri_mf, T1FS_plot, gaussian_mf, singleton_mf
import numpy as np  # add dependencies in cmake/package.xml
import matplotlib.pyplot as plt


class LaneDeviationController:

    def __init__(self):
        """
        CONTROLLER SETUP
        """
        """INPUT AND OUTPUT DOMAINS"""
        self.lane_deviation_domain = linspace(-3, 3, 100)
        steering_angle_domain = linspace(-1, 1, 100)

        """DEFINE INPUTS AND OUTPUT"""
        self.TRJ_CTRL = T1Mamdani(engine="Minimum", defuzzification="CoG")
        self.TRJ_CTRL.add_input_variable("lane_deviation")

        self.TRJ_CTRL.add_output_variable("steering_angle")

        """LANE DEVIATION"""
        left_deviation = T1FS(self.lane_deviation_domain, rtri_mf, [-0.0, -2, 1])
        no_deviation = T1FS(self.lane_deviation_domain, tri_mf, [-0.5, 0, 0.5, 1])
        right_deviation = T1FS(self.lane_deviation_domain, ltri_mf, [0.0, 2, 1])
        # T1FS_plot(left_deviation, no_deviation, right_deviation, legends=("left", "no", "right"))

        """STEERING ANGLE"""
        steering_angle_left = T1FS(steering_angle_domain, tri_mf, [-0.2, -0.1, 0.0, 1])
        steering_angle_straight = T1FS(steering_angle_domain, tri_mf, [-0.05, 0, 0.05, 1])
        steering_angle_right = T1FS(steering_angle_domain, tri_mf, [0.0, 0.1, 0.2, 1])

        # T1FS_plot(steering_angle_left, steering_angle_straight, steering_angle_right,
        # legends=("left", "straight", "right"))

        """FUZZY RULES"""

        self.TRJ_CTRL.add_rule([("lane_deviation", left_deviation)], [("steering_angle", steering_angle_right)])
        self.TRJ_CTRL.add_rule([("lane_deviation", no_deviation)], [("steering_angle", steering_angle_straight)])
        self.TRJ_CTRL.add_rule([("lane_deviation", right_deviation)], [("steering_angle", steering_angle_left)])

        self.create_plot()

    def evaluate(self, lane_deviation):

        something, output_dict = self.TRJ_CTRL.evaluate({
            "lane_deviation": lane_deviation})

        steering_angle = output_dict['steering_angle']

        return steering_angle

    def create_plot(self):

        steering_angle_outputs = []

        for lane_deviation in self.lane_deviation_domain:
            steering_angle_outputs.append(self.evaluate(lane_deviation))

        plt.plot(self.lane_deviation_domain, steering_angle_outputs)

        plt.xlabel("lane deviation (m)")
        plt.ylabel("steering input")
        plt.grid()
        plt.show()


if __name__ == "__main__":
    try:
        LaneDeviationController()
    except Exception as exp:
        print(exp)
