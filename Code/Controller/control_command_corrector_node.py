#!/usr/bin/env python

# CARLA IMPORTS
from carla_msgs.msg import CarlaEgoVehicleControl, \
    CarlaEgoVehicleStatus
from carla_teleoperation_bridge.msg import kf_update_msg
from geometry_msgs.msg import Pose2D, Accel
from nav_msgs.msg import Odometry
import carla

# SELF WRITTEN MODULES
from lane_deviation_calculator import get_lane_deviation
from Test_track_setup import setup_test_track, draw_trajectory_line
from create_test_plots import TestPlot
from helper_functions import (calculate_distance, get_change_in_value, dampen_value,
                              get_heading_error, crossed_line, calculate_lateral_acceleration)

# CONTROLLERS
from DSC_Controller import DSCController
from Heading_Controller import HeadingController
from LaneDeviation_Controller import LaneDeviationController

# OTHER IMPORTS
import rospy
import numpy as np  # add dependencies in cmake/package.xml
import threading
import time
import math


class CommandCorrector:
    def __init__(self):
        # STATE CHECK VARIABLES
        self.trajectory_line_drawn = False
        self.target_changed = False
        self.track_completed = False
        self.crossed_start_line = False
        self.crossed_end_line = False
        self.test_controller_state = True
        self.saving_data = False

        # VEHICLE CONTROL VARIABLES
        self.previous_target_speed = None
        self.test_speed = 8.0
        self.lane_deviation = 0
        self.steering_input = 0
        self.previous_multiplication_factor = 1
        self.previous_tele_operator_steering_angle = 0
        self.speed = 0
        self.max_speed = 0
        self.delay = 0
        self.heading_error = 0
        self.test_max_speed = 0
        self.vehicle_acceleration = 0

        # TEST DATA STORAGE
        self.all_vehicle_points = []
        self.test_vehicle_waypoints = []
        self.test_final_steering_angle_list = []
        self.test_lane_deviation_list = []
        self.test_dsc_steering_angle_list = []
        self.test_heading_steering_angle_list = []
        self.test_lane_deviation_steering_angle_list = []
        self.test_tele_operator_steering_angle_list = []
        self.test_heading_error_list = []
        self.test_speed_list = []
        self.test_lateral_acceleration_list = []

        # GET WORLD AND MAP OBJECTS
        try:
            self.world = carla.Client("localhost", 2000).get_world()
            self.carla_map = self.world.get_map()
        except Exception as exp:
            print("Could not connect. Is Carla server running?")
            print(exp)

        # SETUP TEST TRACK
        self.distance_between_trajectory_waypoints = 0.5
        (self.warning_sign_object, self.cone_object_list,
         self.list_of_trajectory_points, self.list_of_trajectory_waypoints,
         self.start_segment, self.end_segment, self.map_points) = (
            setup_test_track(self.carla_map, self.world, self.distance_between_trajectory_waypoints))

        # CREATE DIRECTED POINTS
        self.list_of_trajectory_directed_points = []
        for i in range(len(self.list_of_trajectory_points) - 1):
            if i == len(self.list_of_trajectory_points) - 1:
                break

            # Current point
            x1, y1 = self.list_of_trajectory_points[i]
            # Next point
            x2, y2 = self.list_of_trajectory_points[i + 1]

            if abs(x1 - x2) < 0.01 and abs(y1 - y2) < 0.01:  # change to if not
                continue

            # Calculate direction vector
            dx = x2 - x1
            dy = y2 - y1
            angle_radians = math.atan2(dy, dx)
            angle_degrees = round(math.degrees(angle_radians), 1)
            self.list_of_trajectory_directed_points.append((x1, y1, angle_degrees))

            # add specific points for lane change
            if -41.7 < x1 < -41.4 and 97 < y1 < 97.2:
                point_amount = 5
                for j in range(1, point_amount):
                    self.list_of_trajectory_directed_points.append((x1 + j * (dx/point_amount),
                                                                    y1 + j * (dy/point_amount),
                                                                    angle_degrees))

        self.trajectory_start_point = self.list_of_trajectory_points[0]
        self.trajectory_end_point = self.list_of_trajectory_points[-1]

        # SUBSCRIBE TO CONTROLLER INPUTS
        # vehicle control command
        self.vehicle_control_command = rospy.Subscriber("/carla_teleop/ego_vehicle/vehicle_control_cmd",
                                                        CarlaEgoVehicleControl, self.vehicle_control_command_callback,
                                                        queue_size=10)
        # vehicle position
        self.dt_ground_truth = rospy.Subscriber("/carla_digital_twin/groundtruth_pose", Pose2D,
                                                self.dt_ground_truth_callback, queue_size=10)

        # vehicle speed
        self.kf_update_subscriber = rospy.Subscriber("/carla_teleop/ego_vehicle/kf_update", kf_update_msg,
                                                     self.kf_update_callback, queue_size=10, tcp_nodelay=True)

        # vehicle acceleration
        self.vehicle_acceleration_subscriber = rospy.Subscriber("/carla_teleop/ego_vehicle/ego_vehicle_acceleration", Accel,
                                                     self.vehicle_acceleration_callback, queue_size=10, tcp_nodelay=True)

        # PUBLISH CONTROL COMMAND WITH MODIFIED STEERING ANGLE
        self.corrected_command_publisher = rospy.Publisher('/vehicle_control_command_corrected',
                                                           CarlaEgoVehicleControl, queue_size=10)

        # INITIALIZE NODE
        rospy.init_node("control_command_corrector_node")

        self.update_frequency = 20
        self.ros_rate = rospy.Rate(self.update_frequency)
        self.vehicle_control_message = CarlaEgoVehicleControl()

        if rospy.has_param('/delay'):
            self.delay = int(rospy.get_param('delay'))
        else:
            print("/delay param not set. default to delay=0")
            self.delay = 0

        # CREATE CONTROLLER INSTANCES
        self.dcs_controller = DSCController()
        self.heading_controller = HeadingController()
        self.lane_deviation_controller = LaneDeviationController()

        # SET WEIGHTS FOR CONTROLLERS (BEST: DSC: 5, HEADING: 1, LANE DEVIATION: 1)
        dsc_weight = 5  
        heading_weight = 1
        lane_deviation_weight = 1
        summed_weights = dsc_weight + heading_weight + lane_deviation_weight

        self.normalized_dsc_weight = dsc_weight / summed_weights
        self.normalized_heading_weight = heading_weight / summed_weights
        self.normalized_lane_deviation_weight = lane_deviation_weight / summed_weights

    def kf_update_callback(self, kf_update_msg):
        self.speed = kf_update_msg.v
        if self.saving_data:
            self.test_speed_list.append(self.speed)
            if self.speed > self.test_max_speed:
                self.test_max_speed = self.speed

    def vehicle_acceleration_callback(self, msg):
        vehicle_yaw = msg.angular.x
        vehicle_acceleration = msg.linear.x

        if self.saving_data:
            lateral_acceleration = calculate_lateral_acceleration(vehicle_acceleration, vehicle_yaw)
            self.test_lateral_acceleration_list.append(lateral_acceleration)

    def vehicle_control_command_callback(self, msg):
        self.vehicle_control_message = msg
        self.steering_input = msg.steer
        tele_operator_speed = msg.target_speed

        if self.saving_data and not self.target_changed:
            self.vehicle_control_message.target_speed = self.test_speed

            if self.previous_target_speed != tele_operator_speed:
                self.target_changed = True
                self.vehicle_control_message.target_speed = tele_operator_speed

        self.previous_target_speed = tele_operator_speed

    def dt_ground_truth_callback(self, msg):
        vehicle_point = (msg.x, msg.y)
        self.all_vehicle_points.append(vehicle_point)
        if self.saving_data:
            self.test_vehicle_waypoints.append(vehicle_point)

        # HEADING ERROR
        vehicle_heading = msg.theta
        heading_error_reference_directed_point = self.get_heading_error_reference_waypoint(vehicle_point)
        next_trajectory_directed_point_heading_rad = heading_error_reference_directed_point[2] * (np.pi / 179)
        self.heading_error = get_heading_error(vehicle_heading, next_trajectory_directed_point_heading_rad)
        if self.saving_data:
            self.test_heading_error_list.append(self.heading_error)

        # LANE DEVIATION FOR EVALUATION
        self.lane_deviation = get_lane_deviation(vehicle_point, self.list_of_trajectory_points)
        if self.list_of_trajectory_points and self.saving_data:
            self.test_lane_deviation_list.append(self.lane_deviation)

    def get_heading_error_reference_waypoint(self, vehicle_point):
        distances = [calculate_distance(vehicle_point, trajectory_point) for trajectory_point
                     in self.list_of_trajectory_points]
        min_distance_index = np.argmin(distances)

        # estimate at which trajectory point the vehicle will be at once the steering command arrives
        index_jump_amount = int(
            round((self.speed * (self.delay / 1000)) / self.distance_between_trajectory_waypoints, 1))

        heading_reference_index = min_distance_index + index_jump_amount
        if heading_reference_index >= len(self.list_of_trajectory_directed_points):
            heading_reference_index = len(self.list_of_trajectory_directed_points) - 1

        heading_error_reference_directed_point = self.list_of_trajectory_directed_points[heading_reference_index]
        return heading_error_reference_directed_point

    def get_controller_steering_angle(self, change_in_steering, speed, delay, current_steering_angle, heading_error,
                                      lane_deviation):
        # HEADING CONTROLLER
        heading_controller_steering_angle = self.heading_controller.evaluate(heading_error)

        # LANE DEVIATION CONTROLLER
        lane_deviation_controller_steering_angle = self.lane_deviation_controller.evaluate(lane_deviation)

        # DCS CONTROLLER
        multiplication_factor = self.dcs_controller.evaluate(delay, speed, change_in_steering)
        dampened_multiplication_factor = dampen_value(self.previous_multiplication_factor, multiplication_factor, 0.01)
        self.previous_multiplication_factor = dampened_multiplication_factor
        shifted_steering_angle = current_steering_angle + 1
        dsc_steering_angle = (shifted_steering_angle * dampened_multiplication_factor) - 1

        final_steering_angle = (self.normalized_dsc_weight * dsc_steering_angle +
                                self.normalized_heading_weight * heading_controller_steering_angle
                                + self.normalized_lane_deviation_weight * lane_deviation_controller_steering_angle)

        if self.saving_data:
            self.test_dsc_steering_angle_list.append(dsc_steering_angle)
            self.test_heading_steering_angle_list.append(heading_controller_steering_angle)
            self.test_lane_deviation_steering_angle_list.append(lane_deviation_controller_steering_angle)

        return final_steering_angle

    def run(self):
        test_start_time = None
        while not rospy.is_shutdown():
            change_in_steering = get_change_in_value(self.steering_input, self.previous_tele_operator_steering_angle)
            self.previous_tele_operator_steering_angle = self.steering_input

            controller_steering_angle = self.get_controller_steering_angle(change_in_steering, self.speed, self.delay,
                                                                           self.steering_input, self.heading_error,
                                                                           self.lane_deviation)

            self.vehicle_control_message.steer = controller_steering_angle

            # if test hasn't started, use tele-operator steering angle
            if not self.saving_data:
                self.test_controller_state = False
                self.vehicle_control_message.steer = self.steering_input
                print("----------")

            # FULL MANUAL CONTROL
            # print("----- MANUAL CONTROL-----")
            self.test_controller_state = False
            self.vehicle_control_message.steer = self.steering_input

            # PUBLISH CORRECTED CONTROL COMMAND
            self.corrected_command_publisher.publish(self.vehicle_control_message)

            # CHECK IF VEHICLE CROSSED START AND END LINES
            if not self.crossed_start_line and len(self.all_vehicle_points) > 3:
                self.crossed_start_line = crossed_line(self.all_vehicle_points[-1],
                                                       self.all_vehicle_points[-4],
                                                       self.start_segment)

            elif len(self.all_vehicle_points) > 3:
                self.crossed_end_line = crossed_line(self.all_vehicle_points[-1],
                                                     self.all_vehicle_points[-4],
                                                     self.end_segment)

            if self.crossed_end_line and self.crossed_start_line:
                print("----- TEST ENDED -----")
                self.saving_data = False
                self.crossed_start_line = False
                self.crossed_end_line = False
                self.target_changed = False
                self.trajectory_line_drawn = False

                # CREATE TEST PLOTS AND SAVE TEST DATA
                test_time = time.time() - test_start_time
                test_state = (self.speed, self.test_controller_state, self.delay)
                TestPlot(test_state, self.map_points, self.list_of_trajectory_points, self.test_vehicle_waypoints,
                         self.test_final_steering_angle_list, self.test_lane_deviation_list,
                         self.test_max_speed, self.test_tele_operator_steering_angle_list,
                         self.test_dsc_steering_angle_list,
                         self.test_heading_steering_angle_list, self.test_lane_deviation_steering_angle_list,
                         self.test_heading_error_list, self.test_speed_list, test_time,
                         self.test_lateral_acceleration_list)

                self.test_vehicle_waypoints = []
                self.test_final_steering_angle_list = []
                self.test_lane_deviation_list = []
                self.test_max_speed = 0

            if self.crossed_start_line:
                self.saving_data = True
                self.test_controller_state = True
                print("----- TESTING -----")
                if not self.trajectory_line_drawn:
                    test_start_time = time.time()
                    # turn on trajectory line
                    draw_trajectory_line(self.world, self.list_of_trajectory_waypoints, carla.Color(0, 255, 0), 300)
                    self.trajectory_line_drawn = True
                    
            if self.saving_data:
                self.test_final_steering_angle_list.append((self.steering_input, controller_steering_angle))
                self.test_tele_operator_steering_angle_list.append(self.steering_input)

            # CHECK DELAY PARAMETER
            if rospy.has_param('/delay'):
                self.delay = int(rospy.get_param('delay'))
            else:
                print("/delay param not set. default to delay=0")
                self.delay = 0

            print(f"cruise control: {self.vehicle_control_message.cruise_control},"
                  f" {self.vehicle_control_message.target_speed} mps")
            print(f"delay: {self.delay} ms")
            print(f"controller state: {self.test_controller_state}")
            # print(f"control loop runtime: {round((time.time() - start_time) * 1000, 2)} ms")
            print("")
            self.ros_rate.sleep()

        # AFTER CONTROLLER STOPS RUNNING
        # DRAW RED TRAJECTORY
        draw_trajectory_line(self.world, self.list_of_trajectory_waypoints, carla.Color(255, 0, 0), 10)

        # REMOVE OBJECTS
        if self.warning_sign_object:
            self.warning_sign_object.destroy()

        for cone_object in self.cone_object_list:
            cone_object.destroy()

        rospy.set_param('/delay', 0)


def main():
    try:
        command_corrector = CommandCorrector()
        command_corrector.run()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
