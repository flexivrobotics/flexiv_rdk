#!/usr/bin/env python

"""basics2_clear_fault.py

This tutorial clears minor faults from the robot server if any. Note that critical faults cannot
be cleared, see RDK manual for more details.
"""

__copyright__ = "Copyright (C) 2016-2023 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import argparse

# Import Flexiv RDK Python library
# fmt: off
import sys
sys.path.insert(0, "../lib_py")
import flexivrdk
# fmt: on


def print_description():
    """
    Print tutorial description.

    """
    print(
        "This tutorial clears minor faults from the robot server if any. Note that "
        "critical faults cannot be cleared, see RDK manual for more details."
    )
    print()


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

    # Print description
    print("[info] Tutorial description:")
    print_description()

    try:
        # RDK Initialization
        # ==========================================================================================
        # Instantiate robot interface
        robot = flexivrdk.Robot(args.robot_sn)

        # Fault Clearing
        # ==========================================================================================
        # Clear fault on the connected robot if any
        if robot.fault():
            print(
                "[warning] Fault occurred on the connected robot, trying to clear ..."
            )
            # Try to clear the fault
            if not robot.ClearFault():
                print("[error] Fault cannot be cleared, exiting ...")
                return 1
            print("[info] Fault on the connected robot is cleared")
        else:
            print("[info] No fault on the connected robot")

    except Exception as e:
        # Print exception error message
        print("[error] ", str(e))


if __name__ == "__main__":
    main()
