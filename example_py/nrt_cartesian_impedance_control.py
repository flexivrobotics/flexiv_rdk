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
sys.path.insert(0, "../lib/")
import flexivrdk
# fmt: on


def main():
    # Parse Arguments
    # =============================================================================
    argparser = argparse.ArgumentParser()
    argparser.add_argument('robot_ip', help='IP address of the robot server')
    argparser.add_argument('local_ip', help='IP address of the workstation PC')
    argparser.add_argument(
        'motion_type', help='Accepted motion types: hold, sine-sweep')
    argparser.add_argument(
        'frequency', help='Command frequency, 1 to 1000 [Hz]')
    args = argparser.parse_args()

    # Convert string to number
    frequency = float(args.frequency)

    # Check if arguments are valid
    assert (args.motion_type == "hold" or args.motion_type ==
            "sine-sweep"), "Invalid <motion_type> input"
    assert (frequency >= 1 and frequency <= 1000), "Invalid <frequency> input"

    # RDK Initialization
    # =============================================================================
    # Some alias
    robot = flexivrdk.Robot()
    mode = flexivrdk.Mode
    robot_states = flexivrdk.RobotStates()
    system_status = flexivrdk.SystemStatus()

    # Initialize connection with robot server
    robot.init(args.robot_ip, args.local_ip)

    # Wait for the connection to be established
    while not robot.isConnected():
        time.sleep(1)

    # Enable the robot, make sure the E-stop is released before enabling
    if robot.enable():
        print("Enabling robot ...")

    # Wait for the robot to become operational
    while not robot.isOperational():
        time.sleep(1)
    print("Robot is now operational")

    # Set mode after robot is operational
    robot.setMode(mode.MODE_CARTESIAN_IMPEDANCE_NRT)

    # Wait for the mode to be switched
    while (robot.getMode() != mode.MODE_CARTESIAN_IMPEDANCE_NRT):
        time.sleep(1)

    # Application-specific Code
    # =============================================================================
    period = 1.0/frequency
    loop_time = 0
    print("Sending command to robot at", frequency,
          "Hz, or", period, "seconds interval")

    # Use current robot TCP pose as initial pose
    robot.getRobotStates(robot_states)
    init_pose = robot_states.m_tcpPose.copy()
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

        # Sine-sweep TCP along y axis
        if args.motion_type == "sine-sweep":
            target_pose[1] = init_pose[1] + SWING_AMP * \
                math.sin(2 * math.pi * SWING_FREQ * loop_time)
        # Otherwise robot TCP will hold at initial pose

        # Send command
        robot.sendTcpPose(target_pose, max_wrench)

        # Increment loop time
        loop_time += period


if __name__ == "__main__":
    main()
