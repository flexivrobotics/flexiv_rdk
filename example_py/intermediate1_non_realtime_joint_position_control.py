#!/usr/bin/env python

"""intermediate1_non_realtime_joint_position_control.py

This tutorial runs non-real-time joint position control to hold or sine-sweep all robot joints.
"""

__copyright__ = "Copyright (C) 2016-2026 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import math
import argparse
import spdlog  # pip install spdlog
import flexivrdk  # pip install flexivrdk


def main():
    # Program Setup
    # ==============================================================================================
    # Parse arguments
    argparser = argparse.ArgumentParser()
    # Required arguments
    argparser.add_argument(
        "robot_sn",
        help="Serial number of the robot to connect. Remove any space, e.g. Enlight-L-123456",
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
        "hold or sine-sweep all robot joints.\n"
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

        # Servo on the robot, make sure the E-stop is released
        logger.info("Servo on the robot ...")
        robot.ServoOn()

        # Wait for the robot to become operational
        while not robot.operational():
            time.sleep(1)

        logger.info("Robot is now operational")

        # Move robot to home pose
        logger.info("Moving to home pose")
        robot.Home()

        # Non-real-time Joint Position Control
        # ==========================================================================================
        # Direct joint control can be executed by single-arm joint groups and the external axis
        exe_groups = robot.info().single_arm_groups
        if not exe_groups:
            raise RuntimeError("No single-arm joint group found on the connected robot")
        # The external axis joint group (if it exists) also supports direct joint control
        if flexivrdk.JointGroup.EXT_AXIS in robot.info().all_groups:
            exe_groups[flexivrdk.JointGroup.EXT_AXIS] = robot.info().all_groups[
                flexivrdk.JointGroup.EXT_AXIS
            ]

        # Switch to non-real-time joint position control mode
        robot.SwitchMode(mode.NRT_JOINT_POSITION)

        period = 1.0 / frequency
        loop_time = 0
        logger.info(
            f"Sending command to robot at {frequency} Hz, or {period} seconds interval"
        )

        # Use current robot joint positions as initial positions
        all_init_pos = {}
        robot_states = robot.states()
        for group in exe_groups:
            all_init_pos[group] = robot_states[group].q.copy()
            logger.info(
                f"[{flexivrdk.kJointGroupNames[group]}] Initial joint positions: {all_init_pos[group]}"
            )

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

            cmds = {}
            sine_offset = SWING_AMP * math.sin(2 * math.pi * SWING_FREQ * loop_time)
            for group, init_pos in all_init_pos.items():
                target_pos = init_pos.copy()
                if not args.hold:
                    for i in range(len(target_pos)):
                        target_pos[i] += sine_offset

                zero_vel = [0.0] * len(init_pos)
                max_vel = [2.0] * len(init_pos)
                max_acc = [3.0] * len(init_pos)
                cmds[group] = flexivrdk.NrtJointPositionCmd(
                    target_pos, zero_vel, max_vel, max_acc
                )

            robot.SendJointPosition(cmds)

            # Increment loop time
            loop_time += period

    except Exception as e:
        # Print exception error message
        logger.error(str(e))
        return 1


if __name__ == "__main__":
    main()
