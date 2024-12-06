#!/usr/bin/env python

"""basics3_primitive_execution.py

This tutorial executes several basic robot primitives (unit skills). For detailed documentation
on all available primitives, please see [Flexiv Primitives](https://www.flexiv.com/primitives/).
"""

__copyright__ = "Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import argparse
import spdlog  # pip install spdlog

# Utility methods
from utility import quat2eulerZYX
from utility import list2str

# Flexiv RDK Python library is installed to user site packages
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
        ">>> Tutorial description <<<\nThis tutorial executes several basic robot primitives (unit "
        "skills). For detailed documentation on all available primitives, please see [Flexiv "
        "Primitives](https://www.flexiv.com/primitives/).\n"
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

        # Execute Primitives
        # ==========================================================================================
        # Switch to primitive execution mode
        robot.SwitchMode(mode.NRT_PRIMITIVE_EXECUTION)

        # (1) Go to home pose
        # ------------------------------------------------------------------------------------------
        # All parameters of the "Home" primitive are optional, thus we can skip the parameters and
        # the default values will be used
        logger.info("Executing primitive: Home")

        # Send command to robot
        robot.ExecutePrimitive("Home", dict())

        # Wait for reached target
        # Note: primitive_states() returns a dictionary of {pt_state_name, [pt_state_values]}
        while not robot.primitive_states()["reachedTarget"]:
            time.sleep(1)

        # (2) Move robot joints to target positions
        # ------------------------------------------------------------------------------------------
        # Required parameters:
        #     target: final joint positions, unit: degrees
        #         {arm joint positions}, {external axis joint positions, optional}
        # Optional parameters:
        #     waypoints: waypoints to pass before reaching the target
        #         (same format as above, but can be more than one)
        #     vel: TCP linear velocity, unit: m/s
        # Optional properties:
        #     lockExternalAxes: whether to allow the external axes to move or not
        logger.info("Executing primitive: MoveJ")

        # Send command to robot
        robot.ExecutePrimitive(
            "MoveJ",
            {
                "target": flexivrdk.JPos(
                    [30, -45, 0, 90, 0, 40, 30], [-50, 30, 0, 0, 0, 0]
                ),
                "waypoints": [
                    flexivrdk.JPos(
                        [10, -30, 10, 30, 10, 15, 10], [-15, 10, 0, 0, 0, 0]
                    ),
                    flexivrdk.JPos(
                        [20, -60, -10, 60, -10, 30, 20], [-30, 20, 0, 0, 0, 0]
                    ),
                ],
            },
            {
                "lockExternalAxes": 0,
            },
        )
        # Most primitives won't exit by themselves and require users to explicitly trigger
        # transitions based on specific primitive states. Here we check if the primitive state
        # [reachedTarget] becomes true and trigger the transition manually by sending a new
        # primitive command.
        while not robot.primitive_states()["reachedTarget"]:
            # Print current primitive states
            print(robot.primitive_states())
            time.sleep(1)

        # (3) Move robot TCP to a target pose in world (base) frame
        # ------------------------------------------------------------------------------------------
        # Required parameters:
        #     target: final TCP pose, unit: m and degrees
        #         {pos_x, pos_y, pos_z}, {rot_x, rot_y, rot_z}, {ref_frame, ref_point}
        # Optional parameters:
        #     waypoints: waypoints to pass before reaching the target
        #         (same format as above, but can be more than one)
        #     vel: TCP linear velocity, unit: m/s
        # NOTE: The rotations use Euler ZYX convention, rot_x means Euler ZYX angle around X axis
        logger.info("Executing primitive: MoveL")

        # Send command to robot
        robot.ExecutePrimitive(
            "MoveL",
            {
                "target": flexivrdk.Coord(
                    [0.65, -0.3, 0.2], [180, 0, 180], ["WORLD", "WORLD_ORIGIN"]
                ),
                "waypoints": [
                    flexivrdk.Coord(
                        [0.45, 0.1, 0.2], [180, 0, 180], ["WORLD", "WORLD_ORIGIN"]
                    ),
                    flexivrdk.Coord(
                        [0.45, -0.3, 0.2], [180, 0, 180], ["WORLD", "WORLD_ORIGIN"]
                    ),
                ],
                "vel": 0.6,
                "zoneRadius": "Z50",
            },
        )
        # Wait for reached target
        while not robot.primitive_states()["reachedTarget"]:
            time.sleep(1)

        # (4) Another MoveL that uses TCP frame
        # ------------------------------------------------------------------------------------------
        # In this example the reference frame is changed from WORLD::WORLD_ORIGIN to TRAJ::START,
        # which represents the current TCP frame
        logger.info("Executing primitive: MoveL")

        # Example to convert target quaternion [w,x,y,z] to Euler ZYX using scipy package's 'xyz'
        # extrinsic rotation
        target_quat = [0.9185587, 0.1767767, 0.3061862, 0.1767767]
        # ZYX = [30, 30, 30] degrees
        eulerZYX_deg = quat2eulerZYX(target_quat, degree=True)

        # Send command to robot. This motion will hold current TCP position and only do rotation
        robot.ExecutePrimitive(
            "MoveL",
            {
                "target": flexivrdk.Coord(
                    [0.0, 0.0, 0.0], eulerZYX_deg, ["TRAJ", "START"]
                ),
                "vel": 0.2,
            },
        )

        # Wait for reached target
        while not robot.primitive_states()["reachedTarget"]:
            time.sleep(1)

        # All done, stop robot and put into IDLE mode
        robot.Stop()

    except Exception as e:
        # Print exception error message
        logger.error(str(e))


if __name__ == "__main__":
    main()
