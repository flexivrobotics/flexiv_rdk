#!/usr/bin/env python

"""basics3_primitive_execution.py

This tutorial executes several basic robot primitives (unit skills). For detailed documentation
on all available primitives, please see [Flexiv Primitives](https://www.flexiv.com/primitives/).
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

        # Servo on the robot, make sure the E-stop is released
        logger.info("Servo on the robot ...")
        robot.ServoOn()

        # Wait for the robot to become operational
        while not robot.operational():
            time.sleep(1)

        logger.info("Robot is now operational")

        # Execute Primitives
        # ==========================================================================================
        # Primitives can only be executed on single-arm joint groups
        single_arm_groups = robot.info().single_arm_groups
        if not single_arm_groups:
            raise RuntimeError("No single-arm joint group found on the connected robot")

        # (1) Move robot to home pose
        # ------------------------------------------------------------------------------------------
        logger.info("Moving to home pose")
        robot.Home()

        # Switch to primitive execution mode
        robot.SwitchMode(mode.NRT_PRIMITIVE_EXECUTION)

        # (2) Move robot joints to target positions
        # ------------------------------------------------------------------------------------------
        # Required parameters:
        #     target: final joint positions, unit: degrees
        #         {arm joint positions}, {external axis joint positions, optional}
        # Optional parameters:
        #     waypoints: waypoints to pass before reaching the target
        #         (same format as above, but can be more than one)
        #     vel: TCP linear velocity, unit: m/s
        logger.info("Executing primitive: MoveJ")

        # Send command to robot
        robot.ExecutePrimitive(
            {
                group: flexivrdk.PrimitiveArgs(
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
                )
                for group in single_arm_groups
            }
        )
        # Most primitives won't exit by themselves and require users to explicitly trigger
        # transitions based on specific primitive states. Here we check if the primitive state
        # [reachedTarget] becomes true and trigger the transition manually by sending a new
        # primitive command.
        while True:
            primitive_states = robot.primitive_states()
            if utility.primitive_state_true_for_groups(
                primitive_states, "reachedTarget"
            ):
                break
            # Print current primitive states
            logger.info("Current primitive states:")
            for group, pt_states in primitive_states.items():
                print(f"{flexivrdk.kJointGroupNames[group]}:")
                print(f"primitiveName: {pt_states.pt_name}")
                for name, value in pt_states.names_and_values.items():
                    print(f"{name}: {value}")
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
            {
                group: flexivrdk.PrimitiveArgs(
                    "MoveL",
                    {
                        "target": flexivrdk.Coord(
                            [0.3, -0.1, 0.2], [160, 20, 180], ["WORLD", "WORLD_ORIGIN"]
                        ),
                        "waypoints": [
                            flexivrdk.Coord(
                                [0.1, 0.1, 0.1],
                                [-160, -20, 180],
                                ["WORLD", "WORLD_ORIGIN"],
                            ),
                            flexivrdk.Coord(
                                [0.3, 0.2, 0.1],
                                [180, 0, 180],
                                ["WORLD", "WORLD_ORIGIN"],
                            ),
                        ],
                        "vel": 0.6,
                        "zoneRadius": "Z50",
                    },
                )
                for group in single_arm_groups
            }
        )
        # Wait for reached target
        while not utility.primitive_state_true_for_groups(
            robot.primitive_states(), "reachedTarget"
        ):
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
        eulerZYX_deg = utility.quat2eulerZYX(target_quat, degree=True)

        # Send command to robot. This motion will hold current TCP position and only do rotation
        robot.ExecutePrimitive(
            {
                group: flexivrdk.PrimitiveArgs(
                    "MoveL",
                    {
                        "target": flexivrdk.Coord(
                            [0.0, 0.0, 0.0], eulerZYX_deg, ["TRAJ", "START"]
                        ),
                        "vel": 0.2,
                    },
                )
                for group in single_arm_groups
            }
        )

        # Wait for reached target
        while not utility.primitive_state_true_for_groups(
            robot.primitive_states(), "reachedTarget"
        ):
            time.sleep(1)

        # All done, stop robot and put into IDLE mode
        robot.Stop()

    except Exception as e:
        # Print exception error message
        logger.error(str(e))
        return 1


if __name__ == "__main__":
    main()
