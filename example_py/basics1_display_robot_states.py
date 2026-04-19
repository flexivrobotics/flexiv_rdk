#!/usr/bin/env python

"""basics1_display_robot_states.py

This tutorial does the very first thing: check connection with the robot server and print
received robot states.
"""

__copyright__ = "Copyright (C) 2016-2026 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import argparse
import threading
import spdlog  # pip install spdlog
import flexivrdk  # pip install flexivrdk


def print_robot_states(robot, logger, stop_event):
    """
    Print robot states data @ 1Hz.

    """

    while not stop_event.is_set():
        # Print available joint groups
        joint_groups_str = " ".join(
            [f"[{flexivrdk.kJointGroupNames[group]}]" for group in robot.groups()]
        )
        logger.info(f"Available joint groups: {joint_groups_str}")

        # Print all robot states in JSON format using the built-in __str__ overloading
        for group, states in robot.states().items():
            logger.info(f"[{flexivrdk.kJointGroupNames[group]}] robot states:")
            # fmt: off
            print("{")
            print(f"timestamp: [{states.timestamp[0]}, {states.timestamp[1]}]")
            print(f"q: {['%.3f' % i for i in states.q]}")
            print(f"theta: {['%.3f' % i for i in states.theta]}")
            print(f"dq: {['%.3f' % i for i in states.dq]}")
            print(f"dtheta: {['%.3f' % i for i in states.dtheta]}")
            print(f"tau: {['%.3f' % i for i in states.tau]}")
            print(f"tau_des: {['%.3f' % i for i in states.tau_des]}")
            print(f"tau_dot: {['%.3f' % i for i in states.tau_dot]}")
            print(f"tau_ext: {['%.3f' % i for i in states.tau_ext]}")
            print(f"tau_interact: {['%.3f' % i for i in states.tau_interact]}")
            print(f"temperature: {['%.3f' % i for i in states.temperature]}")
            print(f"flange_pose: {['%.3f' % i for i in states.flange_pose]}")
            print(f"tcp_pose: {['%.3f' % i for i in states.tcp_pose]}")
            print(f"tcp_twist: {['%.3f' % i for i in states.tcp_twist]}")
            print(f"tcp_wrench: {['%.3f' % i for i in states.tcp_wrench]}")
            print(f"tcp_wrench_local: {['%.3f' % i for i in states.tcp_wrench_local]}")
            print(f"raw_tcp_wrench: {['%.3f' % i for i in states.raw_tcp_wrench]}")
            print(f"raw_tcp_wrench_local: {['%.3f' % i for i in states.raw_tcp_wrench_local]}")
            print(f"raw_ft_sensor: {['%.3f' % i for i in states.raw_ft_sensor]}")
            print("}", flush=True)
            # fmt: on

        # Print digital inputs
        logger.info("Digital inputs:")
        print(robot.digital_inputs())
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
        help="Serial number of the robot to connect. Remove any space, e.g. Rizon4s-123456",
    )
    args = argparser.parse_args()

    # Define alias
    logger = spdlog.ConsoleLogger("Example")
    # Print description
    logger.info(
        ">>> Tutorial description <<<\nThis tutorial does the very first thing: check connection "
        "with the robot server and print received robot states.\n"
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

    except Exception as e:
        # Print exception error message
        logger.error(str(e))
        return 1

    # Print States
    # =============================================================================
    # Thread for printing robot states
    print_thread = threading.Thread(
        target=print_robot_states, args=[robot, logger, stop_event]
    )
    print_thread.start()

    # Use main thread to catch keyboard interrupt and exit thread
    try:
        while not stop_event.is_set():
            time.sleep(0.1)
    except KeyboardInterrupt:
        # Send signal to exit thread
        logger.info("Stopping print thread")
        stop_event.set()

    # Wait for thread to exit
    print_thread.join()
    logger.info("Print thread exited")


if __name__ == "__main__":
    main()
