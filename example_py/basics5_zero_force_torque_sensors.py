#!/usr/bin/env python

"""basics5_zero_force_torque_sensors.py

This tutorial zeros the robot's force and torque sensors, which is a recommended (but not
mandatory) step before any operations that require accurate force/torque measurement.
"""

__copyright__ = "Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import argparse
import spdlog  # pip install spdlog
from utility import list2str

# Flexiv RDK Python library
import flexivrdk


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
    logger.info(
        ">>> Tutorial description <<<\nThis tutorial zeros the robot's force and torque sensors, "
        "which is a recommended (but not mandatory) step before any operations that require "
        "accurate force/torque measurement."
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

        # Zero Sensors
        # ==========================================================================================
        # Get and print the current TCP force/moment readings
        logger.info(
            f"TCP force and moment reading in base frame BEFORE sensor zeroing: {robot.states().ext_wrench_in_world} N-Nm"
        )

        # Run the "ZeroFTSensor" primitive to automatically zero force and torque sensors
        robot.SwitchMode(mode.NRT_PRIMITIVE_EXECUTION)
        robot.ExecutePrimitive("ZeroFTSensor()")

        # WARNING: during the process, the robot must not contact anything, otherwise the result
        # will be inaccurate and affect following operations
        logger.warn(
            "Zeroing force/torque sensors, make sure nothing is in contact with the robot"
        )

        # Wait for the primitive completion
        while robot.busy():
            time.sleep(1)
        logger.info("Sensor zeroing complete")

        # Get and print the current TCP force/moment readings
        logger.info(
            f"TCP force and moment reading in base frame AFTER sensor zeroing: {robot.states().ext_wrench_in_world} N-Nm"
        )

    except Exception as e:
        # Print exception error message
        logger.error(str(e))


if __name__ == "__main__":
    main()
