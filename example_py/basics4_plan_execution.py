#!/usr/bin/env python

"""basics4_plan_execution.py

This tutorial executes a plan selected by the user from a list of available plans. A plan is a
pre-written script to execute a series of robot primitives with pre-defined transition conditions
between 2 adjacent primitives. Users can use Flexiv Elements to compose their own plan and assign
to the robot, which will appear in the plan list.
"""

__copyright__ = "Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import argparse

# Import Flexiv RDK Python library
# fmt: off
import sys
sys.path.insert(0, "../lib_py")
import flexivrdk
# fmt: on


def print_description():
    """
    Print tutorial description.

    """
    print(
        "This tutorial executes a plan selected by the user from a list of available "
        "plans. A plan is a pre-written script to execute a series of robot primitives "
        "with pre-defined transition conditions between 2 adjacent primitives. Users can "
        "use Flexiv Elements to compose their own plan and assign to the robot, which "
        "will appear in the plan list."
    )
    print()


def main():
    # Program Setup
    # ==============================================================================================
    # Parse arguments
    argparser = argparse.ArgumentParser()
    argparser.add_argument("robot_ip", help="IP address of the robot server")
    argparser.add_argument("local_ip", help="IP address of this PC")
    args = argparser.parse_args()

    # Define alias
    log = flexivrdk.Log()
    mode = flexivrdk.Mode
    plan_info = flexivrdk.PlanInfo()

    # Print description
    log.info("Tutorial description:")
    print_description()

    try:
        # RDK Initialization
        # ==========================================================================================
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

        # Execute Plans
        # ==========================================================================================
        # Switch to plan execution mode
        robot.setMode(mode.NRT_PLAN_EXECUTION)

        while True:
            # Monitor fault on robot server
            if robot.isFault():
                raise Exception("Fault occurred on robot server, exiting ...")

            # Get user input
            log.info("Choose an action:")
            print("[1] Show available plans")
            print("[2] Execute a plan by index")
            print("[3] Execute a plan by name")
            user_input = int(input())

            # Get and show plan list
            if user_input == 1:
                plan_list = robot.getPlanNameList()
                for i in range(len(plan_list)):
                    print("[" + str(i) + "]", plan_list[i])
                print("")

            # Execute plan by index
            elif user_input == 2:
                index = int(input("Enter plan index to execute:\n"))
                # Allow the plan to continue its execution even if the RDK program is closed or
                # the connection is lost
                robot.executePlan(index, True)

                # Print plan info while the current plan is running
                while robot.isBusy():
                    robot.getPlanInfo(plan_info)
                    log.info(" ")
                    print("assignedPlanName: ", plan_info.assignedPlanName)
                    print("ptName: ", plan_info.ptName)
                    print("nodeName: ", plan_info.nodeName)
                    print("nodePath: ", plan_info.nodePath)
                    print("nodePathTimePeriod: ", plan_info.nodePathTimePeriod)
                    print("nodePathNumber: ", plan_info.nodePathNumber)
                    print("velocityScale: ", plan_info.velocityScale)
                    print("")
                    time.sleep(1)

            # Execute plan by name
            elif user_input == 3:
                name = str(input("Enter plan name to execute:\n"))
                # Allow the plan to continue its execution even if the RDK program is closed or
                # the connection is lost
                robot.executePlan(name, True)

                # Print plan info while the current plan is running
                while robot.isBusy():
                    robot.getPlanInfo(plan_info)
                    log.info(" ")
                    print("assignedPlanName: ", plan_info.assignedPlanName)
                    print("ptName: ", plan_info.ptName)
                    print("nodeName: ", plan_info.nodeName)
                    print("nodePath: ", plan_info.nodePath)
                    print("nodePathTimePeriod: ", plan_info.nodePathTimePeriod)
                    print("nodePathNumber: ", plan_info.nodePathNumber)
                    print("velocityScale: ", plan_info.velocityScale)
                    print("")
                    time.sleep(1)

            else:
                log.warn("Invalid input")

    except Exception as e:
        # Print exception error message
        log.error(str(e))


if __name__ == "__main__":
    main()
