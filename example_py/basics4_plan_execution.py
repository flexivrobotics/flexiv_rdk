#!/usr/bin/env python

"""basics4_plan_execution.py

This tutorial executes a plan selected by the user from a list of available plans. A plan is a
pre-written script to execute a series of robot primitives with pre-defined transition conditions
between 2 adjacent primitives. Users can use Flexiv Elements to compose their own plan and assign
to the robot, which will appear in the plan list.
"""

__copyright__ = "Copyright (C) 2016-2023 Flexiv Ltd. All Rights Reserved."
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
    argparser.add_argument(
        "robot_sn",
        help="Serial number of the robot to connect to. Remove any space, for example: Rizon4s-123456",
    )
    args = argparser.parse_args()

    # Define alias
    log = flexivrdk.Log()
    mode = flexivrdk.Mode

    # Print description
    log.Info("Tutorial description:")
    print_description()

    try:
        # RDK Initialization
        # ==========================================================================================
        # Instantiate robot interface
        robot = flexivrdk.Robot(args.robot_sn)

        # Clear fault on the connected robot if any
        if robot.fault():
            log.Warn("Fault occurred on the connected robot, trying to clear ...")
            # Try to clear the fault
            if not robot.ClearFault():
                log.Error("Fault cannot be cleared, exiting ...")
                return 1
            log.Info("Fault on the connected robot is cleared")

        # Enable the robot, make sure the E-stop is released before enabling
        log.Info("Enabling robot ...")
        robot.Enable()

        # Wait for the robot to become operational
        while not robot.operational():
            time.sleep(1)

        log.Info("Robot is now operational")

        # Execute Plans
        # ==========================================================================================
        # Switch to plan execution mode
        robot.SwitchMode(mode.NRT_PLAN_EXECUTION)

        while True:
            # Monitor fault on the connected robot
            if robot.fault():
                raise Exception("Fault occurred on the connected robot, exiting ...")

            # Get user input
            log.Info("Choose an action:")
            print("[1] Show available plans")
            print("[2] Execute a plan by index")
            print("[3] Execute a plan by name")
            user_input = int(input())

            # Get and show plan list
            if user_input == 1:
                plan_list = robot.plan_list()
                for i in range(len(plan_list)):
                    print("[" + str(i) + "]", plan_list[i])
                print("")

            # Execute plan by index
            elif user_input == 2:
                index = int(input("Enter plan index to execute:\n"))
                # Allow the plan to continue its execution even if the RDK program is closed or
                # the connection is lost
                robot.ExecutePlan(index, True)

                # Print plan info while the current plan is running
                while robot.busy():
                    plan_info = robot.plan_info()
                    log.Info(" ")
                    print("assigned_plan_name: ", plan_info.assigned_plan_name)
                    print("pt_name: ", plan_info.pt_name)
                    print("node_name: ", plan_info.node_name)
                    print("node_path: ", plan_info.node_path)
                    print("node_path_time_period: ", plan_info.node_path_time_period)
                    print("node_path_number: ", plan_info.node_path_number)
                    print("velocity_scale: ", plan_info.velocity_scale)
                    print("waiting_for_step: ", plan_info.waiting_for_step)
                    print("")
                    time.sleep(1)

            # Execute plan by name
            elif user_input == 3:
                name = str(input("Enter plan name to execute:\n"))
                # Allow the plan to continue its execution even if the RDK program is closed or
                # the connection is lost
                robot.ExecutePlan(name, True)

                # Print plan info while the current plan is running
                while robot.busy():
                    plan_info = robot.plan_info()
                    log.Info(" ")
                    print("assigned_plan_name: ", plan_info.assigned_plan_name)
                    print("pt_name: ", plan_info.pt_name)
                    print("node_name: ", plan_info.node_name)
                    print("node_path: ", plan_info.node_path)
                    print("node_path_time_period: ", plan_info.node_path_time_period)
                    print("node_path_number: ", plan_info.node_path_number)
                    print("velocity_scale: ", plan_info.velocity_scale)
                    print("waiting_for_step: ", plan_info.waiting_for_step)
                    print("")
                    time.sleep(1)

            else:
                log.Warn("Invalid input")

    except Exception as e:
        # Print exception error message
        log.Error(str(e))


if __name__ == "__main__":
    main()
