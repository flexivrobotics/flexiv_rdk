#!/usr/bin/env python

"""primitive_execution.py

Execute various primitives using RDK's primitive execution API.
"""

__copyright__ = "Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import argparse

# Import Flexiv RDK Python library
# fmt: off
import sys
sys.path.insert(0, "../lib/linux/python/x64/")
import flexivrdk
# fmt: on


def main():
    # Parse Arguments
    # =============================================================================
    argparser = argparse.ArgumentParser()
    argparser.add_argument('robot_ip', help='IP address of the robot server')
    argparser.add_argument('local_ip', help='IP address of the workstation PC')
    args = argparser.parse_args()

    # Define alias
    # =============================================================================
    log = flexivrdk.Log()
    mode = flexivrdk.Mode

    try:
        # RDK Initialization
        # =============================================================================
        # Some alias
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
        while not robot.isOperational():
            time.sleep(1)
        log.info("Robot is now operational")

        # Set mode after robot is operational
        robot.setMode(mode.MODE_PRIMITIVE_EXECUTION)

        # Wait for the mode to be switched
        while (robot.getMode() != mode.MODE_PRIMITIVE_EXECUTION):
            time.sleep(1)

        # Application-specific Code
        # =============================================================================
        # Flag to transit to the next primitive
        transit_primitive = False

        # Index for iterating through various primitives
        pt_index = 1

        # Go home first
        robot.executePrimitive("Home()")
        log.info("Homing ...")

        while True:
            # Run user periodic task at 10Hz
            time.sleep(0.1)

            # Monitor fault on robot server
            if robot.isFault():
                raise Exception("Fault occurred on robot server, exiting ...")

            # Reset flag before checking for transition conditions
            transit_primitive = False

            # Get system status which contains primitive states
            pt_states = robot.getPrimitiveStates()

            # Check for transition condition "reachedTarget = 1"
            for state in pt_states:
                print(state)

                # Split the state sentence into words
                words = state.split()

                # Check if this state is "reachedTarget"
                if words[0] == "reachedTarget":
                    # Check if the state value is 1
                    if words[-1] == "1":
                        # Set transition flag
                        transit_primitive = True
                        log.info("Transitting to next primitive ...")

            # Iterate through various primitives if the previous is finished
            if transit_primitive:
                if pt_index == 0:
                    # (1) Go home, all parameters of the "Home" primitive are
                    # optional, thus we can skip the parameters and the default
                    # values will be used
                    robot.executePrimitive("Home()")

                    # Execute the next primitive
                    pt_index += 1

                elif pt_index == 1:
                    # (2) Move TCP to point A in world (base) frame. The
                    # mandatory parameter "target" requires 8 values: [x y z
                    # roll pitch yaw reference_frame reference_point]
                    # unit: meters and degrees
                    robot.executePrimitive(
                        "MoveL(target=0.387 -0.11 0.203 180.0 0.0 180.0 WORLD "
                        "WORLD_ORIGIN)")

                    # Execute the next primitive
                    pt_index += 1

                elif pt_index == 2:
                    # (3) Move TCP to point B in world (base) frame
                    robot.executePrimitive(
                        "MoveL(target=0.687 0.0 0.264 180.0 0.0 180.0 WORLD "
                        "WORLD_ORIGIN)")

                    # Execute the next primitive
                    pt_index += 1

                elif pt_index == 3:
                    # (4) Move TCP to point C in world (base) frame
                    robot.executePrimitive(
                        "MoveL(target=0.387 0.1 0.1 -90.0 0.0 180.0 WORLD "
                        "WORLD_ORIGIN)")

                    # Execute the next primitive
                    pt_index += 1

                elif pt_index == 4:
                    # (5) Move TCP to point D in world(base) frame
                    robot.executePrimitive(
                        "MoveL(target=0.5 0.0 0.3 180.0 0.0 180.0 WORLD "
                        "WORLD_ORIGIN)")

                    # Execute the next primitive
                    pt_index += 1

                elif pt_index == 5:
                    # Repeat the moves
                    pt_index = 0

    except Exception as e:
        # Print exception error message
        log.error(str(e))


if __name__ == "__main__":
    main()
