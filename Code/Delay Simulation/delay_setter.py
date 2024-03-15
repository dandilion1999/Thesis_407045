import subprocess
import sys
import time
import os
import logging

"""
netimpair.py must be in same directory for script to work.

arguments: publisher node, topic, delay in ms
example call: python3 delay_setter.py /control_command_corrector_node /vehicle_control_corrected 100
"""


def execute_command(command):
    # Execute the command and capture the output
    process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    stdout, stderr = process.communicate()

    # Check for errors
    if stderr:
        logging.error(stderr.decode())
    else:
        return stdout.decode()


def get_controller_port_number():
    """GET INPUTS"""
    """SOURCE NODE"""
    try:
        source_node = sys.argv[1]
    except IndexError:
        logging.error(
            "Add source_node as argument behind script call: "
            "...delay_setter.py 'input_source_node' 'input_topic' 'input_delay'")
        return

    """TOPIC"""
    try:
        topic = sys.argv[2]
    except IndexError:
        logging.error(
            "Add topic as argument behind script call: "
            "...delay_setter.py 'input_source_node' 'input_topic' 'input_delay'")
        return

    """DELAY"""
    try:
        delay = sys.argv[3]
    except IndexError:
        logging.error(
            "Add delay as argument behind script call: "
            "...delay_setter.py 'input_source_node' 'input_topic' 'input_delay'")
        return

    if not delay.isnumeric():
        logging.error("delay must be numeric")
        return

    """MAKE SURE NODE IS RUNNING"""
    list_output = execute_command("rosnode list")
    if list_output:
        list_output_lines = list_output.splitlines()
    else:
        logging.error("can't run 'rosnode list' command")
        return

    source_node_found = False
    for line in list_output_lines:
        if source_node in line:
            source_node_found = True

    if not source_node_found:
        logging.error(f"source node '{source_node}' not found")
        logging.error(f"running nodes:")
        logging.error(list_output)
        return

    """GET LINE WITH PORT NUMBER"""
    info_output = execute_command(f"rosnode info {source_node}")
    if info_output:
        info_output_lines = info_output.splitlines()
    else:
        logging.error(f"can't run 'rosnode info {source_node}' command")
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
        logging.error("Topic not found")
        return

    """GET PORT NUMBER"""
    index = relevant_line.find(")")

    if index != -1:
        # Retrieve the five symbols before ")"
        start_index = max(0, index - 5)  # Ensure start_index is not negative
        port_number = relevant_line[start_index:index]
    else:
        logging.error("port number not found")
        return

    """RUN NETIMPAIR"""

    # this command didn't seem to run in the execute command function, workaround with os.system:
    command = f"sudo python3 netimpair.py -n lo --include dport={port_number} netem --delay {delay}" #dst=127.0.0.1,

    logging.info("executed command:")
    logging.info(command)
    print(command, "\n")
    os.system(command)


if __name__ == "__main__":
    get_controller_port_number()
