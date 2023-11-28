#!/usr/bin/env python

"""intermediate2_non_realtime_cartesian_pure_motion_control.py

This tutorial runs non-real-time Cartesian-space pure motion control to hold or sine-sweep the robot
TCP. A simple collision detection is also included.
"""

__copyright__ = "Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import math
import argparse
import numpy as np

# Import Flexiv RDK Python library
# fmt: off
import sys
sys.path.insert(0, "../lib_py")
import flexivrdk
# fmt: on

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


def print_description():
    """
    Print tutorial description.

    """
    print(
        "This tutorial runs non-real-time Cartesian-space pure motion control to hold or "
        "sine-sweep the robot TCP. A simple collision detection is also included."
    )
    print()


def main():
    # Program Setup
    # ==============================================================================================
    # Parse arguments
    argparser = argparse.ArgumentParser()
    # Required arguments
    argparser.add_argument("robot_ip", help="IP address of the robot server")
    argparser.add_argument("local_ip", help="IP address of this PC")
    argparser.add_argument(
        "frequency", help="command frequency, 1 to 100 [Hz]", type=int
    )
    # Optional arguments
    argparser.add_argument(
        "--hold",
        action="store_true",
        help="robot holds current TCP pose, otherwise do a sine-sweep",
    )
    argparser.add_argument(
        "--collision",
        action="store_true",
        help="enable collision detection, robot will stop upon collision",
    )
    args = argparser.parse_args()

    # Check if arguments are valid
    frequency = args.frequency
    assert frequency >= 1 and frequency <= 100, "Invalid <frequency> input"

    # Define alias
    robot_states = flexivrdk.RobotStates()
    log = flexivrdk.Log()
    mode = flexivrdk.Mode

    # Print description
    log.info("Tutorial description:")
    print_description()

    # Print based on arguments
    if args.hold:
        log.info("Robot holding current TCP pose")
    else:
        log.info("Robot running TCP sine-sweep")

    if args.collision:
        log.info("Collision detection enabled")
    else:
        log.info("Collision detection disabled")

    try:
        # RDK Initialization
        # ==========================================================================================
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

        # Move robot to home pose
        log.info("Moving to home pose")
        robot.setMode(mode.NRT_PRIMITIVE_EXECUTION)
        robot.executePrimitive("Home()")

        # Wait for the primitive to finish
        while robot.isBusy():
            time.sleep(1)

        # Zero Force-torque Sensor
        # =========================================================================================
        # IMPORTANT: must zero force/torque sensor offset for accurate force/torque measurement
        robot.executePrimitive("ZeroFTSensor()")

        # WARNING: during the process, the robot must not contact anything, otherwise the result
        # will be inaccurate and affect following operations
        log.warn(
            "Zeroing force/torque sensors, make sure nothing is in contact with the robot"
        )

        # Wait for primitive completion
        while robot.isBusy():
            time.sleep(1)
        log.info("Sensor zeroing complete")

        # Configure Motion Control
        # =========================================================================================
        # The Cartesian motion force modes do pure motion control out of the box, thus nothing
        # needs to be explicitly configured

        # NOTE: motion control always uses robot base frame, while force control can use
        # either base or TCP frame as reference frame

        # Start Pure Motion Control
        # =========================================================================================
        # Switch to non-real-time mode for discrete motion control
        robot.setMode(mode.NRT_CARTESIAN_MOTION_FORCE)

        # Set initial pose to current TCP pose
        robot.getRobotStates(robot_states)
        init_pose = robot_states.tcpPose.copy()
        print(
            "Initial TCP pose set to [position 3x1, rotation (quaternion) 4x1]: ",
            init_pose,
        )

        # Periodic Task
        # =========================================================================================
        # Set loop period
        period = 1.0 / frequency
        loop_counter = 0
        print(
            "Sending command to robot at",
            frequency,
            "Hz, or",
            period,
            "seconds interval",
        )

        # Send command periodically at user-specified frequency
        while True:
            # Use sleep to control loop period
            time.sleep(period)

            # Monitor fault on robot server
            if robot.isFault():
                raise Exception("Fault occurred on robot server, exiting ...")

            # Read robot states
            robot.getRobotStates(robot_states)

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
            robot.sendCartesianMotionForce(target_pose)

            #  Do the following operations in sequence for every 20 seconds
            time_elapsed = loop_counter * period
            # Online change preferred joint positions at 3 seconds
            if time_elapsed % 20.0 == 3.0:
                preferred_jnt_pos = [0.938, -1.108, -1.254, 1.464, 1.073, 0.278, -0.658]
                robot.setNullSpacePosture(preferred_jnt_pos)
                log.info("Preferred joint positions set to: ")
                print(preferred_jnt_pos)
            # Online change stiffness to half of nominal at 6 seconds
            elif time_elapsed % 20.0 == 6.0:
                new_K = np.multiply(robot.info().nominalK, 0.5)
                robot.setCartesianStiffness(new_K)
                log.info("Cartesian stiffness set to: ")
                print(new_K)
            # Online change to another preferred joint positions at 9 seconds
            elif time_elapsed % 20.0 == 9.0:
                preferred_jnt_pos = [-0.938, -1.108, 1.254, 1.464, -1.073, 0.278, 0.658]
                robot.setNullSpacePosture(preferred_jnt_pos)
                log.info("Preferred joint positions set to: ")
                print(preferred_jnt_pos)
            # Online reset stiffness to nominal at 12 seconds
            elif time_elapsed % 20.0 == 12.0:
                robot.resetCartesianStiffness()
                log.info("Cartesian stiffness is reset")
            # Online reset preferred joint positions to nominal at 14 seconds
            elif time_elapsed % 20.0 == 14.0:
                robot.resetNullSpacePosture()
                log.info("Preferred joint positions are reset")
            # Online enable max contact wrench regulation at 16 seconds
            elif time_elapsed % 20.0 == 16.0:
                max_wrench = [10.0, 10.0, 10.0, 2.0, 2.0, 2.0]
                robot.setMaxContactWrench(max_wrench)
                log.info("Max contact wrench set to: ")
                print(max_wrench)
            # Disable max contact wrench regulation at 19 seconds
            elif time_elapsed % 20.0 == 19.0:
                robot.resetMaxContactWrench()
                log.info("Max contact wrench is reset")

            # Simple collision detection: stop robot if collision is detected at
            # end-effector
            if args.collision:
                collision_detected = False
                ext_force = np.array(
                    [
                        robot_states.extWrenchInBase[0],
                        robot_states.extWrenchInBase[1],
                        robot_states.extWrenchInBase[2],
                    ]
                )
                if np.linalg.norm(ext_force) > EXT_FORCE_THRESHOLD:
                    collision_detected = True

                for v in robot_states.tauExt:
                    if abs(v) > EXT_TORQUE_THRESHOLD:
                        collision_detected = True

                if collision_detected:
                    robot.stop()
                    log.warn("Collision detected, stopping robot and exit program ...")
                    return

            # Increment loop counter
            loop_counter += 1

    except Exception as e:
        # Print exception error message
        log.error(str(e))


if __name__ == "__main__":
    main()
