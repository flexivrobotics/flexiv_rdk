#!/usr/bin/env python

"""clear_fault.py

Position and force control with grippers supported by Flexiv, while printing 
gripper states at 1Hz.
"""

__copyright__ = "Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import argparse
import threading

# Import Flexiv RDK Python library
# fmt: off
import sys
sys.path.insert(0, "../lib_py")
import flexivrdk
# fmt: on

# Global flag: whether the gripper control tasks are finished
g_is_done = False


def print_gripper_states(gripper, log):
    """
    Print gripper states data @ 1Hz.

    """

    # Data struct storing gripper states
    gripper_states = flexivrdk.GripperStates()

    while (not g_is_done):
        # Get the latest gripper states
        gripper.getGripperStates(gripper_states)

        # Print all gripper states, round all float values to 2 decimals
        log.info("Current gripper states:")
        print("width: ", round(gripper_states.width, 2))
        print("force: ", round(gripper_states.force, 2))
        print("max_width: ", round(gripper_states.maxWidth, 2))
        print("is_moving: ", gripper_states.isMoving)
        time.sleep(1)


def main():

    # Parse Arguments
    # =============================================================================
    argparser = argparse.ArgumentParser()
    argparser.add_argument(
        'robot_ip', help='IP address of the robot server')
    argparser.add_argument(
        'local_ip', help='IP address of this PC')
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
        seconds_waited = 0
        while not robot.isOperational():
            time.sleep(1)
            seconds_waited += 1
            if seconds_waited == 10:
                log.warn(
                    "Still waiting for robot to become operational, please "
                    "check that the robot 1) has no fault, 2) is booted "
                    "into Auto mode")

        log.info("Robot is now operational")

        # Set mode after robot is operational
        robot.setMode(mode.MODE_PLAN_EXECUTION)

        # Wait for the mode to be switched
        while (robot.getMode() != mode.MODE_PLAN_EXECUTION):
            time.sleep(1)

        robot.executePlanByName("PLAN-Home")
        # Wait for plan to start
        time.sleep(1)
        # Wait for plan to finish
        while robot.isBusy():
            time.sleep(1)

        # Application-specific Code
        # =============================================================================
        # Instantiate gripper
        gripper = flexivrdk.Gripper(robot)

        # Thread for printing gripper states
        print_thread = threading.Thread(
            target=print_gripper_states, args=[gripper, log])
        print_thread.start()

        # Position control test
        log.info("Closing gripper")
        gripper.move(0.01, 0.1, 20)
        time.sleep(2)
        log.info("Opening gripper")
        gripper.move(0.09, 0.1, 20)
        time.sleep(2)

        # Stop tests
        log.info("Closing gripper")
        gripper.move(0.01, 0.1, 20)
        time.sleep(0.5)
        log.info("Stopping gripper")
        gripper.stop()
        time.sleep(2)
        log.info("Closing gripper")
        gripper.move(0.01, 0.1, 20)
        time.sleep(2)
        log.info("Opening gripper")
        gripper.move(0.09, 0.1, 20)
        time.sleep(0.5)
        log.info("Stopping gripper")
        gripper.stop()
        time.sleep(2)

        # Force control test, if available
        gripper_states = flexivrdk.GripperStates()
        gripper.getGripperStates(gripper_states)
        if abs(gripper_states.force) > sys.float_info.epsilon:
            log.info("Gripper running zero force control")
            gripper.grasp(0)
            # Exit after 10 seconds
            time.sleep(10)

        # Finished, exit all threads
        global g_is_done
        g_is_done = True
        log.info("Program finished")
        print_thread.join()

    except Exception as e:
        log.error(str(e))


if __name__ == "__main__":
    main()
