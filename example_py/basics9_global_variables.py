#!/usr/bin/env python

"""basics9_global_variables.py

This tutorial shows how to add, get, update and remove global variables.
"""

__copyright__ = "Copyright (C) 2016-2026 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import argparse
import logging
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
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
    logger = logging.getLogger("Example")

    # Print description
    logger.info(
        ">>> Tutorial description <<<\nThis tutorial shows how to add, get, update and remove "
        "global variables.\n"
    )

    try:
        # RDK Initialization
        # ==========================================================================================
        # Instantiate robot interface
        robot = flexivrdk.Robot(args.robot_sn)

        # Clear fault on the connected robot if any
        if robot.fault():
            logger.warning("Fault occurred on the connected robot, trying to clear ...")
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

        # Instantiate global variable interface
        global_vars = flexivrdk.GlobalVars(robot)

        # Get existing global variables
        # =========================================================================================
        logger.info("Existing global variables and their current values:")
        for key, value in global_vars.values().items():
            print(f"{key}: {value}")

        # Add new global variables
        # =========================================================================================
        # The data type of the initial value decides the data type of the new variable. Adding a
        # global variable requires IDLE mode, which the robot is in after being enabled
        new_vars = {
            "rdk_example_int": 100,
            "rdk_example_double": 100.123,
            "rdk_example_string": "Flexiv",
            "rdk_example_int_vec": [1, 2, 3],
            "rdk_example_double_vec": [1.1, 2.2, 3.3],
            "rdk_example_string_vec": ["Go", "Flexiv", "Go!"],
            "rdk_example_jpos": flexivrdk.JPos([10, 20, 30, 40, 50, 60, 70]),
            "rdk_example_coord": flexivrdk.Coord(
                [0.1, -0.2, 0.3],
                [-90, -45, 120],
                ["WORLD", "WORLD_ORIGIN"],
                [1, 2, 3, 4, 5, 6, 7],
                [10, 20, 0, 0, 0, 0],
            ),
            "rdk_example_coord_array": [
                flexivrdk.Coord([1, 2, 3], [4, 5, 6], ["WORK", "WorkCoord0"]),
                flexivrdk.Coord([3, 2, 1], [180, 0, 180], ["WORLD", "WORLD_ORIGIN"]),
            ],
        }
        logger.info("Adding new global variables")
        for name, value in new_vars.items():
            if global_vars.exist(name):
                logger.warning(f"Global variable [{name}] already exists, removing it first")
                global_vars.Remove(name)
            global_vars.Add(name, value)

        # Update global variables
        # =========================================================================================
        # Updating a global variable is allowed in any control mode, e.g. to feed values to a
        # running plan. Several global variables can be updated in one request
        logger.info("Updating global variables")
        global_vars.Update(
            {
                "rdk_example_int": 200,
                "rdk_example_double": 200.456,
                "rdk_example_int_vec": [4, 5, 6],
            }
        )
        global_vars.Update("rdk_example_string", "Flexiv_Robotics")

        # Get updated global variables
        # =========================================================================================
        logger.info("Added global variables and their updated values:")
        for name in new_vars:
            print(f"{name}: {global_vars.value(name)}")

        # Remove the added global variables
        # =========================================================================================
        # Like adding, removing a global variable requires IDLE mode. Both are saved to the
        # robot's global variable file, so they persist after a power cycle
        logger.info("Removing the added global variables")
        for name in new_vars:
            global_vars.Remove(name)

        logger.info("Program finished")

    except Exception as e:
        # Print exception error message
        logger.error(str(e))


if __name__ == "__main__":
    main()
