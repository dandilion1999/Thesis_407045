from numpy import linspace
from pyit2fls import T1FS, T1Mamdani, rtri_mf, ltri_mf, tri_mf, T1FS_plot, gaussian_mf, singleton_mf
import numpy as np  # add dependencies in cmake/package.xml
import matplotlib.pyplot as plt


class DSCController:

    def __init__(self):
        """
        CONTROLLER SETUP
        """
        """INPUT AND OUTPUT DOMAINS"""
        self.change_in_steering_domain = linspace(-0.4, 0.4, 100)
        self.delay_domain = linspace(0, 500, 100)
        speed_domain = linspace(1, 20, 20)

        multiplication_factor_domain = linspace(0.5, 1.5, 100)  # used for input and output

        """DEFINE INPUTS AND OUTPUT"""
        self.DCS_CTRL = T1Mamdani(engine="Minimum", defuzzification="CoG")
        self.DCS_CTRL.add_input_variable("change_in_steering")
        self.DCS_CTRL.add_input_variable("delay")
        self.DCS_CTRL.add_input_variable("speed")

        self.DCS_CTRL.add_output_variable("multiplication_factor")

        """DELAY"""
        no_delay = T1FS(self.delay_domain, rtri_mf, [100, 0, 1])
        low_delay = T1FS(self.delay_domain, tri_mf, [50, 125, 200, 1])
        high_delay = T1FS(self.delay_domain, ltri_mf, [100, 300, 1])
        #T1FS_plot(no_delay, low_delay, high_delay, legends=("no", "low", "high"))

        """CHANGE IN STEERING"""
        negative_change_in_steering = T1FS(self.change_in_steering_domain, rtri_mf, [-0.0, -0.15, 1])
        no_change_in_steering = T1FS(self.change_in_steering_domain, tri_mf, [-0.01, 0, 0.01, 1])
        positive_change_in_steering = T1FS(self.change_in_steering_domain, ltri_mf, [0.0, 0.15, 1])

        #T1FS_plot(negative_change_in_steering, no_change_in_steering,
        #         positive_change_in_steering, legends=("negative", "no", "large"))

        """SPEED"""
        """speed = T1FS(speed_domain, ltri_mf, [0, 20, 1])
        # T1FS_plot(speed)"""

        """MULTIPLICATION FACTOR"""
        sub_one_multiplication_factor = T1FS(multiplication_factor_domain, tri_mf, [0.9, 0.96, 0.98, 1])
        little_sub_one_multiplication_factor = T1FS(multiplication_factor_domain, tri_mf, [0.96, 0.98, 1, 1])
        one_multiplication_factor = T1FS(multiplication_factor_domain, tri_mf, [0.99, 1, 1.01, 1])
        little_above_one_multiplication_factor = T1FS(multiplication_factor_domain, tri_mf, [1, 1.02, 1.04, 1])
        above_one_multiplication_factor = T1FS(multiplication_factor_domain, tri_mf, [1.02, 1.04, 1.1, 1])
        #T1FS_plot(sub_one_multiplication_factor, little_sub_one_multiplication_factor, one_multiplication_factor,
        #          little_above_one_multiplication_factor,
        #          above_one_multiplication_factor, legends=(
        #        "sub_one_multiplication_factor", "little_sub_one_multiplication_factor", "one_multiplication_factor",
        #        "little_above_one_multiplication_factor",
        #        "above_one_multiplication_factor"))

        """FUZZY RULES"""

        self.DCS_CTRL.add_rule([("delay", no_delay), ("change_in_steering", negative_change_in_steering)],
                               [("multiplication_factor", one_multiplication_factor)])
        self.DCS_CTRL.add_rule([("delay", no_delay), ("change_in_steering", no_change_in_steering)],
                               [("multiplication_factor", one_multiplication_factor)])
        self.DCS_CTRL.add_rule([("delay", no_delay), ("change_in_steering", positive_change_in_steering)],
                               [("multiplication_factor", one_multiplication_factor)])

        self.DCS_CTRL.add_rule([("delay", low_delay), ("change_in_steering", negative_change_in_steering)],
                               [("multiplication_factor", little_sub_one_multiplication_factor)])
        self.DCS_CTRL.add_rule([("delay", low_delay), ("change_in_steering", no_change_in_steering)],
                               [("multiplication_factor", one_multiplication_factor)])
        self.DCS_CTRL.add_rule([("delay", low_delay), ("change_in_steering", positive_change_in_steering)],
                               [("multiplication_factor", little_above_one_multiplication_factor)])

        self.DCS_CTRL.add_rule([("delay", high_delay), ("change_in_steering", negative_change_in_steering)],
                               [("multiplication_factor", sub_one_multiplication_factor)])
        self.DCS_CTRL.add_rule([("delay", high_delay), ("change_in_steering", no_change_in_steering)],
                               [("multiplication_factor", one_multiplication_factor)])
        self.DCS_CTRL.add_rule([("delay", high_delay), ("change_in_steering", positive_change_in_steering)],
                               [("multiplication_factor", above_one_multiplication_factor)])

        self.create_surface()

    def evaluate(self, delay, speed, change_in_steering):

        something, output_dict = self.DCS_CTRL.evaluate({
            "change_in_steering": change_in_steering,
            "delay": delay,
            "speed": speed})

        multiplication_factor = output_dict['multiplication_factor']

        return multiplication_factor

    def create_surface(self):

        changes_in_steering_for_plot, delays_for_plot = np.meshgrid(self.change_in_steering_domain,
                                                                    self.delay_domain)

        multiplication_factor_outputs = np.zeros(shape=(len(self.change_in_steering_domain), len(self.delay_domain)))

        for i, change_in_steering_value in enumerate(self.change_in_steering_domain):
            for j, delay_value in enumerate(self.delay_domain):
                multiplication_factor_outputs[j, i] = self.evaluate(delay_value, 1, change_in_steering_value)

        fig = plt.figure(figsize=(10, 7))
        plot = fig.add_subplot(121, projection="3d")
        plot.set_xlabel("steering rate (1/s)")
        plot.set_ylabel("delay (ms)")
        plot.set_zlabel("steering angle multiplication factor")
        plot.plot_surface(changes_in_steering_for_plot, delays_for_plot, multiplication_factor_outputs, cmap="viridis")

        plt.show()


if __name__ == "__main__":
    try:
        DSCController()
    except Exception as exp:
        print(exp)
