#!/usr/bin/env python

"""basics4_plan_execution.py

This tutorial executes a plan selected by the user from a list of available plans. A plan is a
pre-written script to execute a series of robot primitives with pre-defined transition conditions
between 2 adjacent primitives. Users can use Flexiv Elements to compose their own plan and assign
to the robot, which will appear in the plan list.
"""

__copyright__ = "Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import argparse
import spdlog  # pip install spdlog

# Flexiv RDK Python library
import flexivrdk


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
    logger = spdlog.ConsoleLogger("Example")
    mode = flexivrdk.Mode

    # Print description
    logger.info(
        ">>> Tutorial description <<<\nThis tutorial executes a plan selected by the user from a "
        "list of available plans. A plan is a pre-written script to execute a series of robot "
        "primitives with pre-defined transition conditions between 2 adjacent primitives. Users "
        "can use Flexiv Elements to compose their own plan and assign to the robot, which "
        "will appear in the plan list."
    )

    try:
        # RDK Initialization
        # ==========================================================================================
        # Instantiate robot interface
        robot = flexivrdk.Robot(args.robot_sn)

        # Clear fault on the connected robot if any
        if robot.fault():
            logger.warn("Fault occurred on the connected robot, trying to clear ...")
            # Try to clear the fault
            if not robot.ClearFault():
                logger.error("Fault cannot be cleared, exiting ...")
                return 1
            logger.info("Fault on the connected robot is cleared")

        # Enable the robot, make sure the E-stop is released before enabling
        logger.info("Enabling robot ...")
        robot.Enable()

        # Wait for the robot to become operational
        while not robot.operational():
            time.sleep(1)

        logger.info("Robot is now operational")

        # Execute Plans
        # ==========================================================================================
        # Switch to plan execution mode
        robot.SwitchMode(mode.NRT_PLAN_EXECUTION)

        while True:
            # Monitor fault on the connected robot
            if robot.fault():
                raise Exception("Fault occurred on the connected robot, exiting ...")

            # Get user input
            logger.info("Choose an action:")
            print("[1] Show available plans")
            print("[2] Execute a plan by index")
            print("[3] Execute a plan by name")
            user_input = int(input())

            # Get and show plan list
            if user_input == 1:
                plan_list = robot.plan_list()
                for i in range(len(plan_list)):
                    print(f"[{i}] {plan_list[i]}")
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
                    logger.info("Current plan info:")
                    print(f"assigned_plan_name: {plan_info.assigned_plan_name}")
                    print(f"pt_name: {plan_info.pt_name}")
                    print(f"node_name: {plan_info.node_name}")
                    print(f"node_path: {plan_info.node_path}")
                    print(f"node_path_time_period: {plan_info.node_path_time_period}")
                    print(f"node_path_number: {plan_info.node_path_number}")
                    print(f"velocity_scale: {plan_info.velocity_scale}")
                    print(f"waiting_for_step: {plan_info.waiting_for_step}")
                    print("", flush=True)
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
                    logger.info("Current plan info:")
                    print(f"assigned_plan_name: {plan_info.assigned_plan_name}")
                    print(f"pt_name: {plan_info.pt_name}")
                    print(f"node_name: {plan_info.node_name}")
                    print(f"node_path: {plan_info.node_path}")
                    print(f"node_path_time_period: {plan_info.node_path_time_period}")
                    print(f"node_path_number: {plan_info.node_path_number}")
                    print(f"velocity_scale: {plan_info.velocity_scale}")
                    print(f"waiting_for_step: {plan_info.waiting_for_step}")
                    print("", flush=True)
                    time.sleep(1)

            else:
                logger.warn("Invalid input")

    except Exception as e:
        # Print exception error message
        logger.error(str(e))


if __name__ == "__main__":
    main()
