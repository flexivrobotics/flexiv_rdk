#!/usr/bin/env python

"""basics5_zero_force_torque_sensors.py

This tutorial zeros the robot's force and torque sensors, which is a recommended (but not
mandatory) step before any operations that require accurate force/torque measurement.
"""

__copyright__ = "Copyright (C) 2016-2026 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import argparse
import spdlog  # pip install spdlog
import flexivrdk  # pip install flexivrdk
import utility


def main():
    # Program Setup
    # ==============================================================================================
    # Parse arguments
    argparser = argparse.ArgumentParser()
    argparser.add_argument(
        "robot_sn",
        help="Serial number of the robot to connect. Remove any space, e.g. Enlight-L-123456",
    )
    args = argparser.parse_args()

    # Define alias
    logger = spdlog.ConsoleLogger("Example")
    mode = flexivrdk.Mode

    # Print description
    logger.info(
        ">>> Tutorial description <<<\nThis tutorial zeros the robot's force and torque sensors, "
        "which is a recommended (but not mandatory) step before any operations that require "
        "accurate force/torque measurement.\n"
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

        # Zero Sensors
        # ==========================================================================================
        # Get and print the current TCP force/moment readings
        for group, states in robot.states().items():
            logger.info(
                f"[{flexivrdk.kJointGroupNames[group]}] TCP force and moment reading in world frame BEFORE sensor zeroing: {states.tcp_wrench} N-Nm"
            )

        # Primitives can only be executed on single-arm joint groups
        single_arm_groups = robot.info().single_arm_groups
        if not single_arm_groups:
            raise RuntimeError("No single-arm joint group found on the connected robot")

        # Run the "ZeroFTSensor" primitive to automatically zero force and torque sensors
        robot.SwitchMode(mode.NRT_PRIMITIVE_EXECUTION)
        robot.ExecutePrimitive(
            {
                group: flexivrdk.PrimitiveArgs("ZeroFTSensor", dict())
                for group in single_arm_groups
            }
        )

        # WARNING: during the process, the robot must not contact anything, otherwise the result
        # will be inaccurate and affect following operations
        logger.warn(
            "Zeroing force/torque sensors, make sure nothing is in contact with the robot"
        )

        # Wait for primitive to finish
        while not utility.primitive_state_true_for_groups(robot.primitive_states(), "terminated"):
            time.sleep(1)
        logger.info("Sensor zeroing complete")

        # Get and print the current TCP force/moment readings
        for group, states in robot.states().items():
            logger.info(
                f"[{flexivrdk.kJointGroupNames[group]}] TCP force and moment reading in world frame AFTER sensor zeroing: {states.tcp_wrench} N-Nm"
            )

    except Exception as e:
        # Print exception error message
        logger.error(str(e))
        return 1


if __name__ == "__main__":
    main()
