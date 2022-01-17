#!/usr/bin/env python

"""plan_execution.py

Select a plan from a list to execute using RDK's plan execution API.
"""

__copyright__ = "Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import argparse

# Import Flexiv RDK Python library
# fmt: off
import sys
sys.path.insert(0, "../lib/")
import flexivrdk
# fmt: on


def main():
    # Parse Arguments
    # =============================================================================
    argparser = argparse.ArgumentParser()
    argparser.add_argument('robot_ip', help='IP address of the robot server')
    argparser.add_argument('local_ip', help='IP address of the workstation PC')
    args = argparser.parse_args()

    # RDK Initialization
    # =============================================================================
    # Some alias
    robot = flexivrdk.Robot()
    mode = flexivrdk.Mode
    system_status = flexivrdk.SystemStatus()

    # Initialize connection with robot server
    robot.init(args.robot_ip, args.local_ip)

    # Wait for the connection to be established
    while not robot.isConnected():
        time.sleep(1)

    # Enable the robot, make sure the E-stop is released before enabling
    if robot.enable():
        print("Enabling robot ...")

    # Wait for the robot to become operational
    while not robot.isOperational():
        time.sleep(1)
    print("Robot is now operational")

    # Set mode after robot is operational
    robot.setMode(mode.MODE_PLAN_EXECUTION)

    # Wait for the mode to be switched
    while (robot.getMode() != mode.MODE_PLAN_EXECUTION):
        time.sleep(1)

    # Application-specific Code
    # =============================================================================
    while True:
        # Get user input
        input_case = int(input(
            "Choose an action:\n \
        [1] Get plan list \n \
        [2] Execute plan by index \n \
        [3] Stop the current plan execution \n"
        )
        )

        # Check if user input is valid
        assert (input_case >= 1 and input_case <= 3), "Invalid input"

        # Get plan list
        if input_case == 1:
            plan_list = robot.getPlanNameList()
            time.sleep(1)
            for i in range(len(plan_list)):
                print("[" + str(i) + "]", plan_list[i])

        # Execute plan by index
        elif input_case == 2:
            index = int(input("Enter plan index to execute:\n"))
            robot.executePlanByIndex(index)

        # Stop the current plan execution
        elif input_case == 3:
            robot.stop()


if __name__ == "__main__":
    main()
