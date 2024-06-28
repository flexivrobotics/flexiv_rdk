#!/usr/bin/env python

"""basics6_gripper_control.py

This tutorial does position and force control (if available) for grippers supported by Flexiv.
"""

__copyright__ = "Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import argparse
import threading
import spdlog  # pip install spdlog

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
        "This tutorial does position and force control (if available) for grippers "
        "supported by Flexiv."
    )
    print()


def print_gripper_states(gripper, logger, stop_event):
    """
    Print gripper states data @ 1Hz.

    """
    while not stop_event.is_set():
        # Print all gripper states, round all float values to 2 decimals
        logger.info("Current gripper states:")
        print(f"width: {round(gripper.states().width, 2)}")
        print(f"force: {round(gripper.states().force, 2)}")
        print(f"max_width: {round(gripper.states().max_width, 2)}")
        print(f"moving: {gripper.moving()}")
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
    args = argparser.parse_args()

    # Define alias
    logger = spdlog.ConsoleLogger("Example")
    mode = flexivrdk.Mode

    # Print description
    logger.info("Tutorial description:")
    print_description()

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
        # Gripper control is not available if the robot is in IDLE mode, so switch to some mode
        # other than IDLE
        robot.SwitchMode(mode.NRT_PLAN_EXECUTION)
        robot.ExecutePlan("PLAN-Home")
        time.sleep(1)

        # Instantiate gripper control interface
        gripper = flexivrdk.Gripper(robot)

        # Manually initialize the gripper, not all grippers need this step
        logger.info("Initializing gripper, this process takes about 10 seconds ...")
        gripper.Init()
        logger.info("Initialization complete")

        # Thread for printing gripper states
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
