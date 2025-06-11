#!/usr/bin/env python

"""basics7_auto_recovery.py

This tutorial runs an automatic recovery process if the robot's safety system is in recovery
state. See Robot::recovery() and RDK manual for more details.
"""

__copyright__ = "Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import argparse
import spdlog  # pip install spdlog
import flexivrdk  # pip install flexivrdk


def main():
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
        ">>> Tutorial description <<<\nThis tutorial runs an automatic recovery process if the "
        "robot's safety system is in recovery state. See flexiv::rdk::Robot::recovery() and RDK "
        "manual for more details.\n"
    )

    try:
        # RDK Initialization
        # ==========================================================================================
        # Instantiate robot interface
        robot = flexivrdk.Robot(args.robot_sn)

        # Enable the robot, make sure the E-stop is released before enabling
        logger.info("Enabling robot ...")
        robot.Enable()

        # Run Auto-recovery
        # ==========================================================================================
        # If the system is in recovery state, we can't use isOperational to tell if the enabling
        # process is done, so just wait long enough for the process to finish
        time.sleep(8)

        # Run automatic recovery if the system is in recovery state, the involved joints will start
        # to move back into allowed position range
        if robot.recovery():
            robot.RunAutoRecovery()
        # Otherwise the system is normal, do nothing
        else:
            logger.info(
                "Robot system is not in recovery state, nothing to be done, exiting ..."
            )

    except Exception as e:
        # Print exception error message
        logger.error(str(e))


if __name__ == "__main__":
    main()
