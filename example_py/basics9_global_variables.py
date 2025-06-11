#!/usr/bin/env python

"""basics9_global_variables.py

This tutorial shows how to get and set global variables.
"""

__copyright__ = "Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

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
    mode = flexivrdk.Mode

    # Print description
    logger.info(
        ">>> Tutorial description <<<\nThis tutorial shows how to get and set global variables.\n"
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

        # Get existing global variables
        # =========================================================================================
        global_vars = robot.global_variables()
        if not global_vars:
            logger.warn("No global variables available")
            return 1
        else:
            logger.info("Existing global variables and their original values:")
            for key, value in global_vars.items():
                print(f"{key}: {value}")

        # Set global variables
        # =========================================================================================
        # WARNING: These specified global variables need to be created first using Flexiv Elements
        logger.info("Setting new values to existing global variables")
        robot.SetGlobalVariables(
            {
                "test_bool": 1,
                "test_int": 100,
                "test_double": 100.123,
                "test_string": "Flexiv",
                "test_int_vec": [1, 2, 3],
                "test_double_vec": [1.1, 2.2, 3.3],
                "test_string_vec": ["Go", "Flexiv", "Go!"],
                "test_pose": [0.1, -0.2, 0.3, -90, -45, 120],
                "test_coord": flexivrdk.Coord(
                    [0.1, -0.2, 0.3],
                    [-90, -45, 120],
                    ["WORLD", "WORLD_ORIGIN"],
                    [1, 2, 3, 4, 5, 6, 7],
                    [10, 20, 0, 0, 0, 0],
                ),
                "test_coord_array": [
                    flexivrdk.Coord([1, 2, 3], [4, 5, 6], ["WORK", "WorkCoord0"]),
                    flexivrdk.Coord(
                        [10, 20, 30],
                        [40, 50, 60],
                        ["WORLD", "WORLD_ORIGIN"],
                        [1, 2, 3, 4, 5, 6, 7],
                        [10, 20, 0, 0, 0, 0],
                    ),
                    flexivrdk.Coord(
                        [3, 2, 1], [180, 0, 180], ["WORLD", "WORLD_ORIGIN"]
                    ),
                ],
            }
        )

        # Get updated global variables
        # =========================================================================================
        global_vars = robot.global_variables()
        if not global_vars:
            logger.warn("No global variables available")
            return 1
        else:
            logger.info("Updated global variables:")
            for key, value in global_vars.items():
                print(f"{key}: {value}")

        logger.info("Program finished")

    except Exception as e:
        # Print exception error message
        logger.error(str(e))


if __name__ == "__main__":
    main()
