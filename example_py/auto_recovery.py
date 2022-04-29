#!/usr/bin/env python

"""auto_recovery.py

Run auto-recovery if the robot's safety system is in recovery state,
otherwise run damped floating with joint soft limit disabled, so that the
user can manually trigger a safety system recovery state by moving any joint
close to its limit. See flexiv::Robot::isRecoveryState() for more details.
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
    log = flexivrdk.Log()
    mode = flexivrdk.Mode

    try:
        # RDK Initialization
        # =============================================================================
        # Instantiate robot interface
        robot = flexivrdk.Robot(args.robot_ip, args.local_ip)

        # Enable the robot, make sure the E-stop is released before enabling
        log.info("Enabling robot ...")
        robot.enable()

        # Application-specific Code
        # =============================================================================
        # If the system is in recovery state, we can't use isOperational to tell if
        # the enabling process is done, so just wait long enough for the process to
        # finish
        time.sleep(8)

        # Start auto recovery if the system is in recovery state, the involved
        # joints will start to move back into allowed position range
        if robot.isRecoveryState():
            robot.startAutoRecovery()
            # block forever, must reboot the robot and restart user program after auto
            # recovery is done
            while True:
                time.sleep(1)
        # Otherwise the system is normal, do nothing
        else:
            log.warn(
                "Robot system is not in recovery state, nothing to be done, exiting ...")

    except Exception as e:
        # Print exception error message
        log.error(str(e))


if __name__ == "__main__":
    main()
