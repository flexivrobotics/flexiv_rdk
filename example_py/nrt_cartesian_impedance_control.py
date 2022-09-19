#!/usr/bin/env python

"""nrt_cartesian_impedance_control.py

Run non-real-time Cartesian impedance control to hold or sine-sweep the robot TCP.
"""

__copyright__ = "Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import math
import argparse

# Import Flexiv RDK Python library
# fmt: off
import sys
sys.path.insert(0, "../lib_py")
import flexivrdk
# fmt: on


def main():
    # Parse Arguments
    # =============================================================================
    argparser = argparse.ArgumentParser()
    # Required arguments
    argparser.add_argument("robot_ip", help="IP address of the robot server")
    argparser.add_argument("local_ip", help="IP address of this PC")
    argparser.add_argument(
        "frequency", help="command frequency, 1 to 1000 [Hz]", type=float)
    # Optional arguments
    argparser.add_argument(
        "--hold", action="store_true",
        help="robot holds current TCP pose, otherwise do a sine-sweep")
    args = argparser.parse_args()

    # Check if arguments are valid
    frequency = args.frequency
    assert (frequency >= 1 and frequency <= 1000), "Invalid <frequency> input"

    # Define alias
    # =============================================================================
    robot_states = flexivrdk.RobotStates()
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
        robot.setMode(mode.MODE_CARTESIAN_IMPEDANCE_NRT)

        # Wait for the mode to be switched
        while (robot.getMode() != mode.MODE_CARTESIAN_IMPEDANCE_NRT):
            time.sleep(1)

        # Application-specific Code
        # =============================================================================
        period = 1.0/frequency
        loop_counter = 0
        print("Sending command to robot at", frequency,
              "Hz, or", period, "seconds interval")

        # Use current robot TCP pose as initial pose
        robot.getRobotStates(robot_states)
        init_pose = robot_states.tcpPose.copy()
        print("Initial pose set to: ", init_pose)

        # Initialize target vectors
        target_pose = init_pose.copy()
        max_wrench = [100.0, 100.0, 100.0, 30.0, 30.0, 30.0]

        # TCP sine-sweep amplitude [m]
        SWING_AMP = 0.1

        # TCP sine-sweep frequency [Hz]
        SWING_FREQ = 0.3

        # Send command periodically at user-specified frequency
        while True:
            # Use sleep to control loop period
            time.sleep(period)

            # Monitor fault on robot server
            if robot.isFault():
                raise Exception("Fault occurred on robot server, exiting ...")

            # Sine-sweep TCP along y axis
            if not args.hold:
                target_pose[1] = init_pose[1] + SWING_AMP * \
                    math.sin(2 * math.pi * SWING_FREQ * loop_counter * period)
            # Otherwise robot TCP will hold at initial pose

            # Online change stiffness to softer at 5 seconds
            if ((loop_counter * period) % 10.0 == 5.0):
                new_kp = [2000, 2000, 2000, 200, 200, 200]
                robot.setCartesianStiffness(new_kp)
                log.info("Cartesian stiffness set to: ")
                print(new_kp)

            # Online reset stiffness to original at 9 seconds
            if ((loop_counter * period) % 10.0 == 9.0):
                default_kp = [4000, 4000, 4000, 1900, 1900, 1900]
                robot.setCartesianStiffness(default_kp)
                log.info("Cartesian stiffness is reset to: ")
                print(default_kp)

            # Send command
            robot.sendTcpPose(target_pose, max_wrench)

            # Increment loop counter
            loop_counter += 1

    except Exception as e:
        # Print exception error message
        log.error(str(e))


if __name__ == "__main__":
    main()
