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
sys.path.insert(0, "../lib/linux/python/x64/")
import flexivrdk
# fmt: on


def main():
    # Parse Arguments
    # =============================================================================
    argparser = argparse.ArgumentParser()
    argparser.add_argument('robot_ip', help='IP address of the robot server')
    argparser.add_argument('local_ip', help='IP address of the workstation PC')
    args = argparser.parse_args()

    # Define alias
    # =============================================================================
    system_status = flexivrdk.SystemStatus()
    log = flexivrdk.Log()
    mode = flexivrdk.Mode

    try:
        # RDK Initialization
        # =============================================================================
        # Instantiate robot interface
        robot = flexivrdk.Robot(args.robot_ip, args.local_ip)

        # Clear fault on robot server if any
        if robot.isFault():
            log.warn("Fault occurred on robot server, trying to clear ...")
            # Try to clear the fault
            robot.clearFault()
            time.sleep(2)
            # Check again
            if robot.isFault():
                log.error("Fault cannot be cleared, exiting ...")
                return
            log.info("Fault on robot server is cleared")

        # Enable the robot, make sure the E-stop is released before enabling
        log.info("Enabling robot ...")
        robot.enable()

        # Wait for the robot to become operational
        while not robot.isOperational():
            time.sleep(1)
        log.info("Robot is now operational")

        # Set mode after robot is operational
        robot.setMode(mode.MODE_PLAN_EXECUTION)

        # Wait for the mode to be switched
        while (robot.getMode() != mode.MODE_PLAN_EXECUTION):
            time.sleep(1)

        # Application-specific Code
        # =============================================================================
        while True:
            # Monitor fault on robot server
            if robot.isFault():
                raise Exception("Fault occurred on robot server, exiting ...")

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

    except Exception as e:
        # Print exception error message
        log.error(str(e))


if __name__ == "__main__":
    main()
