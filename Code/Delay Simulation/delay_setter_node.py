#!/usr/bin/env python3
import rospy

import os
import subprocess


def execute_command(command):
    # Execute the command and capture the output
    process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    stdout, stderr = process.communicate()

    # Check for errors
    if stderr:
        logging.error(stderr.decode())
    else:
        return stdout.decode()


def set_delay(source_node, topic, delay):

    """MAKE SURE NODE IS RUNNING"""
    list_output = execute_command("rosnode list")
    if list_output:
        list_output_lines = list_output.splitlines()
    else:
        print("ERROR - can't run 'rosnode list' command")
        rospy.set_param('/delay', 0)
        return

    source_node_found = False
    for line in list_output_lines:
        if source_node in line:
            source_node_found = True

    if not source_node_found:
        print(f"ERROR - source node '{source_node}' not found")
        print(f"running nodes: ", "\n", list_output)
        rospy.set_param('/delay', 0)
        return

    """GET LINE WITH PORT NUMBER"""
    info_output = execute_command(f"rosnode info {source_node}")
    if info_output:
        info_output_lines = info_output.splitlines()
    else:
        print(f"ERROR - can't run 'rosnode info {source_node}' command")
        rospy.set_param('/delay', 0)
        return

    topic_found = False
    relevant_line = ""
    for line in info_output_lines:
        if topic_found:
            if "direction" in line:
                relevant_line = line
                break
        if f"topic: {topic}" in line:
            topic_found = True

    if not relevant_line:
        print("ERROR - Topic not found")
        rospy.set_param('/delay', 0)
        return

    """GET PORT NUMBER"""
    index = relevant_line.find(")")

    if index != -1:
        # Retrieve the five symbols before ")"
        start_index = max(0, index - 5)  # Ensure start_index is not negative
        port_number = relevant_line[start_index:index]
    else:

        print("ERROR - port number not found")
        rospy.set_param('/delay', 0)
        return

    """RUN NETIMPAIR"""

    # this command didn't seem to run in the execute command function, workaround with os.system:

    # use: eno1 and dst=... for two pc setup
    command = f"sudo python3 src/control_command_corrector/src/netimpair.py -n eno1 --include dst=130.149.219.26,dport={port_number} netem --delay {delay}"

    # use this for one pc setup
    # command = f"sudo python3 src/control_command_corrector/src/netimpair.py -n lo --include dport={port_number} netem --delay {delay}" # dst=127.0.0.1,

    print(command, "\n")
    os.system(command)


class DelaySetter:

    def __init__(self):
        rospy.init_node('delay_setter', anonymous=True)

        delay_input = input("Enter delay in ms: ")

        try:
            int(delay_input)
        except ValueError:
            print("ERROR - delay must be of type int")
            return

        rospy.set_param('/delay', delay_input)
        rospy.loginfo("Delay set to ROS parameter '/delay': %s", delay_input)

        source_node = "/control_command_corrector_node"
        topic = "/vehicle_control_command_corrected"

        set_delay(source_node, topic, delay_input)


if __name__ == "__main__":
    try:
        DelaySetter()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.set_param('/delay', 0)
