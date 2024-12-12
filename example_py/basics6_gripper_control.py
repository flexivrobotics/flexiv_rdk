#!/usr/bin/env python

"""basics6_gripper_control.py

This tutorial does position and force (if available) control of grippers supported by Flexiv.
"""

__copyright__ = "Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import argparse
import threading
import spdlog  # pip install spdlog

# Flexiv RDK Python library is installed to user site packages
import flexivrdk


def print_gripper_states(gripper, logger, stop_event):
    """
    Print gripper states data @ 1Hz.

    """
    while not stop_event.is_set():
        # Print all gripper states, round all float values to 2 decimals
        logger.info("Current gripper states:")
        print(f"width: {round(gripper.states().width, 2)}")
        print(f"force: {round(gripper.states().force, 2)}")
        print(f"is_moving: {gripper.states().is_moving}")
        print("", flush=True)
        time.sleep(1)


def main():
    # Create an event to signal the thread to stop
    stop_event = threading.Event()

    # Program Setup
    # ==============================================================================================
    # Parse arguments
    argparser = argparse.ArgumentParser()
    argparser.add_argument(
        "robot_sn",
        help="Serial number of the robot to connect to. Remove any space, for example: Rizon4s-123456",
    )
    argparser.add_argument(
        "gripper_name",
        help="Full name of the gripper to be controlled, can be found in Flexiv Elements -> Settings -> Device",
    )
    args = argparser.parse_args()

    # Define alias
    logger = spdlog.ConsoleLogger("Example")
    mode = flexivrdk.Mode

    # Print description
    logger.info(
        ">>> Tutorial description <<<\nThis tutorial does position and force (if available) "
        "control of grippers supported by Flexiv.\n"
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

        # Gripper Control
        # ==========================================================================================
        # Instantiate gripper control interface
        gripper = flexivrdk.Gripper(robot)

        # Instantiate tool interface. Gripper is categorized as both a device and a tool. The
        # device attribute allows a gripper to be interactively controlled by the user; whereas the
        # tool attribute tells the robot to account for its mass properties and TCP location.
        tool = flexivrdk.Tool(robot)

        # Enable the specified gripper as a device. This is equivalent to enabling the specified
        # gripper in Flexiv Elements -> Settings -> Device
        logger.info(f"Enabling gripper [{args.gripper_name}]")
        gripper.Enable(args.gripper_name)

        # Print parameters of the enabled gripper
        logger.info("Gripper params:")
        print(f"name: {gripper.params().name}")
        print(f"min_width: {round(gripper.params().min_width, 3)}")
        print(f"max_width: {round(gripper.params().max_width, 3)}")
        print(f"min_force: {round(gripper.params().min_force, 3)}")
        print(f"max_force: {round(gripper.params().max_force, 3)}")
        print(f"min_vel: {round(gripper.params().min_vel, 3)}")
        print(f"max_vel: {round(gripper.params().max_vel, 3)}")
        print("", flush=True)

        # Switch robot tool to gripper so the gravity compensation and TCP location is updated
        logger.info(f"Switching robot tool to [{args.gripper_name}]")
        tool.Switch(args.gripper_name)

        # User needs to determine if this gripper requires manual initialization
        logger.info(
            "Manually trigger initialization for the gripper now? Choose Yes if it's a 48v Grav "
            "gripper"
        )
        print("[1] No, it has already initialized automatically when power on")
        print("[2] Yes, it does not initialize itself when power on")
        choice = int(input(""))

        # Trigger manual initialization based on input
        if choice == 1:
            logger.info("Skipped manual initialization")
        elif choice == 2:
            gripper.Init()
            # User determines if the manual initialization is finished
            logger.info(
                "Triggered manual initialization, press Enter when the initialization is finished to continue"
            )
            input()
        else:
            logger.error("Invalid choice")
            return 1

        # Start a separate thread to print gripper states
        print_thread = threading.Thread(
            target=print_gripper_states, args=[gripper, logger, stop_event]
        )
        print_thread.start()

        # Position control
        logger.info("Closing gripper")
        gripper.Move(0.01, 0.1, 20)
        time.sleep(2)
        logger.info("Opening gripper")
        gripper.Move(0.09, 0.1, 20)
        time.sleep(2)

        # Stop
        logger.info("Closing gripper")
        gripper.Move(0.01, 0.1, 20)
        time.sleep(0.5)
        logger.info("Stopping gripper")
        gripper.Stop()
        time.sleep(2)
        logger.info("Closing gripper")
        gripper.Move(0.01, 0.1, 20)
        time.sleep(2)
        logger.info("Opening gripper")
        gripper.Move(0.09, 0.1, 20)
        time.sleep(0.5)
        logger.info("Stopping gripper")
        gripper.Stop()
        time.sleep(2)

        # Force control, if available (sensed force is not zero)
        if abs(gripper.states().force) > sys.float_info.epsilon:
            logger.info("Gripper running zero force control")
            gripper.Grasp(0)
            # Exit after 10 seconds
            time.sleep(10)

        # Finished
        gripper.Stop()

        # Stop all threads
        logger.info("Stopping print thread")
        stop_event.set()
        print_thread.join()
        logger.info("Print thread exited")
        logger.info("Program finished")

    except Exception as e:
        logger.error(str(e))


if __name__ == "__main__":
    main()
