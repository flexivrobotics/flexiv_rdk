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
        "This tutorial does the very first thing: check connection with the robot server "
        "and print received robot states."
    )
    print()


def print_robot_states(robot, logger):
    """
    Print robot states data @ 1Hz.

    """

    while True:
        # Print all gripper states, round all float values to 2 decimals
        logger.info("Current robot states:")
        # fmt: off
        print("{")
        print("q: ",  ['%.2f' % i for i in robot.states().q])
        print("theta: ", ['%.2f' % i for i in robot.states().theta])
        print("dq: ", ['%.2f' % i for i in robot.states().dq])
        print("dtheta: ", ['%.2f' % i for i in robot.states().dtheta])
        print("tau: ", ['%.2f' % i for i in robot.states().tau])
        print("tau_des: ", ['%.2f' % i for i in robot.states().tau_des])
        print("tau_dot: ", ['%.2f' % i for i in robot.states().tau_dot])
        print("tau_ext: ", ['%.2f' % i for i in robot.states().tau_ext])
        print("tcp_pose: ", ['%.2f' % i for i in robot.states().tcp_pose])
        print("tcp_pose_d: ", ['%.2f' % i for i in robot.states().tcp_pose_des])
        print("tcp_velocity: ", ['%.2f' % i for i in robot.states().tcp_vel])
        print("flange_pose: ", ['%.2f' % i for i in robot.states().flange_pose])
        print("FT_sensor_raw_reading: ", ['%.2f' % i for i in robot.states().ft_sensor_raw])
        print("F_ext_tcp_frame: ", ['%.2f' % i for i in robot.states().ext_wrench_in_tcp])
        print("F_ext_world_frame: ", ['%.2f' % i for i in robot.states().ext_wrench_in_world])
        print("}")
        # fmt: on
        time.sleep(1)


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

        # Print States
        # =============================================================================
        # Thread for printing robot states
        print_thread = threading.Thread(target=print_robot_states, args=[robot, logger])
        print_thread.start()
        print_thread.join()

    except Exception as e:
        # Print exception error message
        logger.error(str(e))


if __name__ == "__main__":
    main()
