#!/usr/bin/env python

"""basics1_display_robot_states.py

This tutorial does the very first thing: check connection with the robot server and print 
received robot states.
"""

__copyright__ = "Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import argparse
import threading
import spdlog  # pip install spdlog

# Flexiv RDK Python library is installed to user site packages
import flexivrdk


def print_robot_states(robot, logger, stop_event):
    """
    Print robot states data @ 1Hz.

    """

    while not stop_event.is_set():
        # Print all gripper states, round all float values to 2 decimals
        logger.info("Current robot states:")
        # fmt: off
        print("{")
        print(f"q: {['%.2f' % i for i in robot.states().q]}",)
        print(f"theta: {['%.2f' % i for i in robot.states().theta]}")
        print(f"dq: {['%.2f' % i for i in robot.states().dq]}")
        print(f"dtheta: {['%.2f' % i for i in robot.states().dtheta]}")
        print(f"tau: {['%.2f' % i for i in robot.states().tau]}")
        print(f"tau_des: {['%.2f' % i for i in robot.states().tau_des]}")
        print(f"tau_dot: {['%.2f' % i for i in robot.states().tau_dot]}")
        print(f"tau_ext: {['%.2f' % i for i in robot.states().tau_ext]}")
        print(f"tcp_pose: {['%.2f' % i for i in robot.states().tcp_pose]}")
        print(f"tcp_pose_d: {['%.2f' % i for i in robot.states().tcp_pose_des]}")
        print(f"tcp_velocity: {['%.2f' % i for i in robot.states().tcp_vel]}")
        print(f"flange_pose: {['%.2f' % i for i in robot.states().flange_pose]}")
        print(f"FT_sensor_raw_reading: {['%.2f' % i for i in robot.states().ft_sensor_raw]}")
        print(f"F_ext_tcp_frame: {['%.2f' % i for i in robot.states().ext_wrench_in_tcp]}")
        print(f"F_ext_world_frame: {['%.2f' % i for i in robot.states().ext_wrench_in_world]}")
        print("}", flush= True)
        # fmt: on
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
