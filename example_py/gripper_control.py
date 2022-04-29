#!/usr/bin/env python

"""clear_fault.py

Position and force control with grippers that use the communication protocol
template provided by Flexiv.
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
    argparser.add_argument(
        'robot_ip', help='IP address of the robot server')
    argparser.add_argument(
        'local_ip', help='IP address of the workstation PC')
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

        # Set mode after robot is operational
        robot.setMode(mode.MODE_PLAN_EXECUTION)

        # Wait for the mode to be switched
        while (robot.getMode() != mode.MODE_PLAN_EXECUTION):
            time.sleep(1)

        robot.executePlanByName("PLAN-Home")
        time.sleep(1)

        # Application-specific Code
        # =============================================================================
        # Instantiate gripper
        gripper = flexivrdk.Gripper(robot)
        # Position control
        # Close to width = 0.02m, using velocity = 0.1m/s
        log.info("Closing fingers")
        gripper.move(0.02, 0.1)
        time.sleep(1)
        # Open to width = 0.08m, using velocity = 0.1m/s
        log.info("Opening fingers")
        gripper.move(0.08, 0.1)
        time.sleep(1)

        # Force control
        # Close fingers with 10N
        log.info("Grasping with constant force")
        gripper.grasp(10)
        # Hold for 3 seconds
        time.sleep(3)

        # Open fingers and stop halfway
        log.info("Opening fingers")
        gripper.move(0.08, 0.1)
        time.sleep(0.5)
        log.info("Stopping gripper")
        gripper.stop()

    except Exception as e:
        log.error(str(e))


if __name__ == "__main__":
    main()
