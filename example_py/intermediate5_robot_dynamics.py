#!/usr/bin/env python

"""intermediate5_robot_dynamics.py

This tutorial runs the integrated dynamics engine to obtain robot Jacobian, mass matrix, and
gravity torques. Also checks reachability of a Cartesian pose.
"""

__copyright__ = "Copyright (C) 2016-2026 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
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
        help="Serial number of the robot to connect. Remove any space, e.g. Enlight-L-123456",
    )
    args = argparser.parse_args()

    # Define alias
    logger = spdlog.ConsoleLogger("Example")
    mode = flexivrdk.Mode

    # Print description
    logger.info(
        ">>> Tutorial description <<<\nThis tutorial runs the integrated dynamics engine to obtain "
        "robot Jacobian, mass matrix, and gravity torques. Also checks reachability of a Cartesian "
        "pose.\n"
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

        # Robot Dynamics
        # ==========================================================================================
        # Initialize dynamics engine
        model = flexivrdk.Model(robot)

        # Step dynamics engine 5 times
        for i in range(5):
            # Update robot model in dynamics engine
            robot_states = robot.states()
            model.Update(
                robot_states[flexivrdk.JointGroup.ALL].q,
                robot_states[flexivrdk.JointGroup.ALL].dq,
            )

            # Compute gravity vector
            g = model.g()

            # Compute mass matrix
            M = model.M()

            # Compute Jacobian
            J = model.J("flange")

            # Print result
            logger.info("g = ")
            print(g, flush=True)
            logger.info("M = ")
            print(M, flush=True)
            logger.info("J = ")
            print(J, flush=True)
            print()

        # Check IK feasibility for a nearby Cartesian pose on all available single-arm joint groups
        single_arm_groups = robot.info().single_arm_groups
        if not single_arm_groups:
            raise RuntimeError("No single-arm joint group found on the connected robot")

        robot_states = robot.states()
        ik_params_by_group = {}
        for group in single_arm_groups:
            pose_to_check = robot_states[group].tcp_pose.copy()
            pose_to_check[0] += 0.1
            logger.info(
                f"[{flexivrdk.kJointGroupNames[group]}] Checking IK feasibility of Cartesian pose {pose_to_check}"
            )
            ik_params = flexivrdk.IKParams()
            ik_params.cartesian_pose = pose_to_check
            ik_params.seed_q = robot_states[group].q.copy()
            ik_params.free_orientation = False
            ik_params_by_group[group] = ik_params

        # Print result
        result = model.SolveConstrainedIK(ik_params_by_group)
        logger.info(f"IK result success = {result.success}")
        for group, q in result.solved_q.items():
            logger.info(f"[{flexivrdk.kJointGroupNames[group]}] solved_q = {q}")

    except Exception as e:
        logger.error(str(e))
        return 1


if __name__ == "__main__":
    main()
