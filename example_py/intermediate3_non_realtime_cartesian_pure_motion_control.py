#!/usr/bin/env python

"""intermediate3_non_realtime_cartesian_pure_motion_control.py

This tutorial runs non-real-time Cartesian-space pure motion control to hold or sine-sweep the robot
TCP. A simple collision detection is also included.
"""

__copyright__ = "Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import math
import argparse
import spdlog  # pip install spdlog
import numpy as np  # pip install numpy

# Flexiv RDK Python library is installed to user site packages
import flexivrdk

# Global constants
# ==================================================================================================
# TCP sine-sweep amplitude [m]
SWING_AMP = 0.1

# TCP sine-sweep frequency [Hz]
SWING_FREQ = 0.3

# External TCP force threshold for collision detection, value is only for demo purpose [N]
EXT_FORCE_THRESHOLD = 10.0

# External joint torque threshold for collision detection, value is only for demo purpose [Nm]
EXT_TORQUE_THRESHOLD = 5.0


def main():
    # Program Setup
    # ==============================================================================================
    # Parse arguments
    argparser = argparse.ArgumentParser()
    # Required arguments
    argparser.add_argument(
        "robot_sn",
        help="Serial number of the robot to connect. Remove any space, e.g. Rizon4s-123456",
    )
    argparser.add_argument(
        "frequency", help="Command frequency, 1 to 100 [Hz]", type=int
    )
    # Optional arguments
    argparser.add_argument(
        "--hold",
        action="store_true",
        help="Robot holds current TCP pose, otherwise do a sine-sweep",
    )
    argparser.add_argument(
        "--collision",
        action="store_true",
        help="Enable collision detection, robot will stop upon collision",
    )
    args = argparser.parse_args()

    # Check if arguments are valid
    frequency = args.frequency
    assert frequency >= 1 and frequency <= 100, "Invalid <frequency> input"

    # Define alias
    logger = spdlog.ConsoleLogger("Example")
    mode = flexivrdk.Mode

    # Print description
    logger.info(
        ">>> Tutorial description <<<\nThis tutorial runs non-real-time Cartesian-space pure "
        "motion control to hold or sine-sweep the robot TCP. A simple collision detection is also "
        "included.\n"
    )

    # Print based on arguments
    if args.hold:
        logger.info("Robot holding current TCP pose")
    else:
        logger.info("Robot running TCP sine-sweep")

    if args.collision:
        logger.info("Collision detection enabled")
    else:
        logger.info("Collision detection disabled")

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

        # Zero Force-torque Sensor
        # =========================================================================================
        robot.SwitchMode(mode.NRT_PRIMITIVE_EXECUTION)
        # IMPORTANT: must zero force/torque sensor offset for accurate force/torque measurement
        robot.ExecutePrimitive("ZeroFTSensor", dict())

        # WARNING: during the process, the robot must not contact anything, otherwise the result
        # will be inaccurate and affect following operations
        logger.warn(
            "Zeroing force/torque sensors, make sure nothing is in contact with the robot"
        )

        # Wait for primitive completion
        while robot.busy():
            time.sleep(1)
        logger.info("Sensor zeroing complete")

        # Configure Motion Control
        # =========================================================================================
        # The Cartesian motion force modes do pure motion control out of the box, thus nothing
        # needs to be explicitly configured

        # NOTE: motion control always uses robot world frame, while force control can use
        # either world or TCP frame as reference frame

        # Start Pure Motion Control
        # =========================================================================================
        # Switch to non-real-time mode for discrete motion control
        robot.SwitchMode(mode.NRT_CARTESIAN_MOTION_FORCE)

        # Set all Cartesian axis(s) to motion control
        robot.SetForceControlAxis([False, False, False, False, False, False])

        # Set initial pose to current TCP pose
        init_pose = robot.states().tcp_pose.copy()

        # Save initial joint positions
        init_q = robot.states().q.copy()

        # Periodic Task
        # =========================================================================================
        # Set loop period
        period = 1.0 / frequency
        loop_counter = 0
        logger.info(
            f"Sending command to robot at {frequency} Hz, or {period} seconds interval"
        )

        # Send command periodically at user-specified frequency
        while True:
            # Use sleep to control loop period
            time.sleep(period)

            # Monitor fault on the connected robot
            if robot.fault():
                raise Exception("Fault occurred on the connected robot, exiting ...")

            # Initialize target pose to initial pose
            target_pose = init_pose.copy()

            # Sine-sweep TCP along Y axis
            if not args.hold:
                target_pose[1] = init_pose[1] + SWING_AMP * math.sin(
                    2 * math.pi * SWING_FREQ * loop_counter * period
                )
            # Otherwise robot TCP will hold at initial pose

            # Send command. Calling this method with only target pose input results
            # in pure motion control
            robot.SendCartesianMotionForce(target_pose)

            #  Do the following operations in sequence for every 20 seconds
            time_elapsed = loop_counter * period
            # Online change reference joint positions at 3 seconds
            if time_elapsed % 20.0 == 3.0:
                preferred_jnt_pos = [0.938, -1.108, -1.254, 1.464, 1.073, 0.278, -0.658]
                robot.SetNullSpacePosture(preferred_jnt_pos)
                logger.info(f"Reference joint positions set to {preferred_jnt_pos}")
            # Online change stiffness to half of nominal at 6 seconds
            elif time_elapsed % 20.0 == 6.0:
                new_K = np.multiply(robot.info().K_x_nom, 0.5)
                robot.SetCartesianImpedance(new_K)
                logger.info(f"Cartesian stiffness set to {new_K}")
            # Online change to another reference joint positions at 9 seconds
            elif time_elapsed % 20.0 == 9.0:
                preferred_jnt_pos = [-0.938, -1.108, 1.254, 1.464, -1.073, 0.278, 0.658]
                robot.SetNullSpacePosture(preferred_jnt_pos)
                logger.info(f"Reference joint positions set to {preferred_jnt_pos}")
            # Online reset impedance properties to nominal at 12 seconds
            elif time_elapsed % 20.0 == 12.0:
                robot.SetCartesianImpedance(robot.info().K_x_nom)
                logger.info("Cartesian impedance properties are reset")
            # Online reset reference joint positions to nominal at 14 seconds
            elif time_elapsed % 20.0 == 14.0:
                robot.SetNullSpacePosture(init_q)
                logger.info("Reference joint positions are reset")
            # Online enable max contact wrench regulation at 16 seconds
            elif time_elapsed % 20.0 == 16.0:
                max_wrench = [10.0, 10.0, 10.0, 2.0, 2.0, 2.0]
                robot.SetMaxContactWrench(max_wrench)
                logger.info(f"Max contact wrench set to {max_wrench}")
            # Disable max contact wrench regulation at 19 seconds
            elif time_elapsed % 20.0 == 19.0:
                robot.SetMaxContactWrench([float("inf")] * 6)
                logger.info("Max contact wrench regulation is disabled")

            # Simple collision detection: stop robot if collision is detected at
            # end-effector
            if args.collision:
                collision_detected = False
                ext_force = np.array(
                    [
                        robot.states().ext_wrench_in_world[0],
                        robot.states().ext_wrench_in_world[1],
                        robot.states().ext_wrench_in_world[2],
                    ]
                )
                if np.linalg.norm(ext_force) > EXT_FORCE_THRESHOLD:
                    collision_detected = True

                for v in robot.states().tau_ext:
                    if abs(v) > EXT_TORQUE_THRESHOLD:
                        collision_detected = True

                if collision_detected:
                    robot.Stop()
                    logger.warn(
                        "Collision detected, stopping robot and exit program ..."
                    )
                    return

            # Increment loop counter
            loop_counter += 1

    except Exception as e:
        # Print exception error message
        logger.error(str(e))


if __name__ == "__main__":
    main()
