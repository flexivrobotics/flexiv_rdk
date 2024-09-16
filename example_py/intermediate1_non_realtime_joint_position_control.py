#!/usr/bin/env python

"""intermediate1_non_realtime_joint_position_control.py

This tutorial runs non-real-time joint position control to hold or sine-sweep all robot joints.
"""

__copyright__ = "Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import math
import argparse
import spdlog  # pip install spdlog

# Flexiv RDK Python library is installed to user site packages
import flexivrdk


def main():
    # Program Setup
    # ==============================================================================================
    # Parse arguments
    argparser = argparse.ArgumentParser()
    # Required arguments
    argparser.add_argument(
        "robot_sn",
        help="Serial number of the robot to connect to. Remove any space, for example: Rizon4s-123456",
    )
    argparser.add_argument(
        "frequency", help="Command frequency, 1 to 100 [Hz]", type=int
    )
    # Optional arguments
    argparser.add_argument(
        "--hold",
        action="store_true",
        help="Robot holds current joint positions, otherwise do a sine-sweep",
    )
    args = argparser.parse_args()

    # Check if arguments are valid
    frequency = args.frequency
    assert frequency >= 1 and frequency <= 100, "Invalid <frequency> input"

    # Define alias
    logger = spdlog.ConsoleLogger("Example")
    mode = flexivrdk.Mode

    # Print description
    logger.info(
        ">>> Tutorial description <<<\nThis tutorial runs non-real-time joint position control to "
        "hold or sine-sweep all robot joints."
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

        # Move robot to home pose
        logger.info("Moving to home pose")
        robot.SwitchMode(mode.NRT_PLAN_EXECUTION)
        robot.ExecutePlan("PLAN-Home")
        # Wait for the plan to finish
        while robot.busy():
            time.sleep(1)

        # Non-real-time Joint Position Control
        # ==========================================================================================
        # Switch to non-real-time joint position control mode
        robot.SwitchMode(mode.NRT_JOINT_POSITION)

        period = 1.0 / frequency
        loop_time = 0
        logger.info(
            f"Sending command to robot at {frequency} Hz, or {period} seconds interval"
        )

        # Use current robot joint positions as initial positions
        init_pos = robot.states().q.copy()
        logger.info(f"Initial positions set to: {init_pos}")

        # Robot joint degrees of freedom
        DoF = robot.info().DoF

        # Initialize target vectors
        target_pos = init_pos.copy()
        target_vel = [0.0] * DoF
        target_acc = [0.0] * DoF

        # Joint motion constraints
        MAX_VEL = [2.0] * DoF
        MAX_ACC = [3.0] * DoF

        # Joint sine-sweep amplitude [rad]
        SWING_AMP = 0.1

        # TCP sine-sweep frequency [Hz]
        SWING_FREQ = 0.3

        # Send command periodically at user-specified frequency
        while True:
            # Use sleep to control loop period
            time.sleep(period)

            # Monitor fault on the connected robot
            if robot.fault():
                raise Exception("Fault occurred on the connected robot, exiting ...")

            # Sine-sweep all joints
            if not args.hold:
                for i in range(DoF):
                    target_pos[i] = init_pos[i] + SWING_AMP * math.sin(
                        2 * math.pi * SWING_FREQ * loop_time
                    )
            # Otherwise all joints will hold at initial positions

            # Send command
            robot.SendJointPosition(
                target_pos, target_vel, target_acc, MAX_VEL, MAX_ACC
            )

            # Increment loop time
            loop_time += period

    except Exception as e:
        # Print exception error message
        logger.error(str(e))


if __name__ == "__main__":
    main()
