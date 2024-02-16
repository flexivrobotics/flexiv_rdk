#!/usr/bin/env python

"""basics8_update_robot_tool.py

This tutorial shows how to online update and interact with the robot tools. All changes made to 
the robot tool system will take effect immediately without needing to reboot. However, the robot 
must be put into IDLE mode when making these changes.
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
        "This tutorial shows how to online update and interact with the robot tools. All "
        "changes made to the robot tool system will take effect immediately without needing to "
        "reboot. However, the robot must be put into IDLE mode when making these changes."
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

    # Define alias
    log = flexivrdk.Log()
    mode = flexivrdk.Mode

    # Print description
    log.info("Tutorial description:")
    print_description()

    try:
        # RDK Initialization
        # ==========================================================================================
        # Instantiate robot interface
        robot = flexivrdk.Robot(args.robot_sn)

        # Clear fault on the connected robot if any
        if robot.isFault():
            log.warn("Fault occurred on the connected robot, trying to clear ...")
            # Try to clear the fault
            if not robot.clearFault():
                log.error("Fault cannot be cleared, exiting ...")
                return 1
            log.info("Fault on the connected robot is cleared")

        # Enable the robot, make sure the E-stop is released before enabling
        log.info("Enabling robot ...")
        robot.enable()

        # Wait for the robot to become operational
        while not robot.isOperational():
            time.sleep(1)

        log.info("Robot is now operational")

        # Update Robot Tool
        # ==========================================================================================
        # Make sure the robot is in IDLE mode
        robot.setMode(mode.IDLE)

        # Instantiate tool interface
        tool = flexivrdk.Tool(robot)

        # Get and print a list of already configured tools currently in the robot's tools pool
        log.info("All configured tools:")
        tool_list = tool.getToolList()
        for i in range(len(tool_list)):
            print("[" + str(i) + "]", tool_list[i])
        print()

        # Get and print the current active tool
        log.info("Current active tool: " + tool.getCurrentToolName())

        # Set name and parameters for a new tool
        newToolName = "ExampleTool1"
        newToolParams = flexivrdk.ToolParams()
        newToolParams.mass = 0.9
        newToolParams.CoM = [0.0, 0.0, 0.057]
        newToolParams.inertia = [2.768e-03, 3.149e-03, 5.64e-04, 0.0, 0.0, 0.0]
        newToolParams.tcpLocation = [0.0, -0.207, 0.09, 0.7071068, 0.7071068, 0.0, 0.0]

        # If there's already a tool with the same name in the robot's tools pool, then remove it
        # first, because duplicate tool names are not allowed
        if tool.isExist(newToolName):
            log.warn(
                "Tool with the same name ["
                + newToolName
                + "] already exists, removing it now"
            )
            tool.remove(newToolName)

        # Add the new tool
        log.info("Adding new tool [" + newToolName + "] to the robot")
        tool.add(newToolName, newToolParams)

        # Get and print the tools list again, the new tool should appear at the end
        log.info("All configured tools:")
        tool_list = tool.getToolList()
        for i in range(len(tool_list)):
            print("[" + str(i) + "]", tool_list[i])
        print()

        # Switch to the newly added tool, i.e. set it as the active tool
        log.info("Switching to tool [" + newToolName + "]")
        tool.switchTo(newToolName)

        # Get and print the current active tool again, should be the new tool
        log.info("Current active tool: " + tool.getCurrentToolName())

        # Clean up by removing the new tool
        time.sleep(2)
        log.info("Removing tool [" + newToolName + "]")
        tool.remove(newToolName)

        log.info("Program finished")

    except Exception as e:
        log.error(str(e))


if __name__ == "__main__":
    main()
