#!/usr/bin/env python

"""intermediate5_robot_dynamics.py

This tutorial runs the integrated dynamics engine to obtain robot Jacobian, mass matrix, and
gravity torques. Also checks reachability of a Cartesian pose.
"""

__copyright__ = "Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved."
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
        help="Serial number of the robot to connect. Remove any space, e.g. Rizon4s-123456",
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

        # Enable the robot, make sure the E-stop is released before enabling
        logger.info("Enabling robot ...")
        robot.Enable()

        # Wait for the robot to become operational
        while not robot.operational():
            time.sleep(1)

        logger.info("Robot is now operational")

        # Move robot to home pose
        logger.info("Moving to home pose")
        robot.SwitchMode(mode.NRT_PLAN_EXECUTION)
        robot.ExecutePlan("PLAN-Home")
        # Wait for the plan to finish
        while robot.busy():
            time.sleep(1)

        # Robot Dynamics
        # ==========================================================================================
        # Initialize dynamics engine
        model = flexivrdk.Model(robot)

        # Step dynamics engine 5 times
        for i in range(5):
            # Update robot model in dynamics engine
            model.Update(robot.states().q, robot.states().dtheta)

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

        # Check reachability of a Cartesian pose based on current pose
        pose_to_check = robot.states().tcp_pose.copy()
        pose_to_check[0] += 0.1
        logger.info(f"Checking reachability of Cartesian pose {pose_to_check}")
        result = model.reachable(pose_to_check, robot.states().q, True)
        logger.info(f"Got a result: reachable = {result[0]}, IK solution = {result[1]}")

    except Exception as e:
        logger.error(str(e))


if __name__ == "__main__":
    main()
