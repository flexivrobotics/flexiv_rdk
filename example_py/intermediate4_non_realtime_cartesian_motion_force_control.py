#!/usr/bin/env python

"""intermediate4_non_realtime_cartesian_motion_force_control.py

This tutorial runs non-real-time Cartesian-space unified motion-force control. The Z axis of the
chosen reference frame will be activated for explicit force control, while the rest axes in the
same reference frame will stay motion controlled.
"""

__copyright__ = "Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import math
import argparse
import spdlog  # pip install spdlog
import numpy as np  # pip install numpy
import flexivrdk  # pip install flexivrdk


# Global constants
# ==================================================================================================
# TCP sine-sweep amplitude [m]
SWING_AMP = 0.1

# TCP sine-sweep frequency [Hz]
SWING_FREQ = 0.3

# Pressing force to apply during the unified motion-force control [N]
PRESSING_FORCE = 5.0

# Cartesian linear velocity used to search for contact [m/s]
SEARCH_VELOCITY = 0.02

# Maximum distance to travel when searching for contact [m]
SEARCH_DISTANCE = 1.0

# Maximum contact wrench during contact search for soft contact
MAX_WRENCH_FOR_CONTACT_SEARCH = [10.0, 10.0, 10.0, 3.0, 3.0, 3.0]


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
        "--TCP",
        action="store_true",
        help="Use TCP frame as reference frame for force control, otherwise use world frame",
    )
    argparser.add_argument(
        "--polish",
        action="store_true",
        help="Run a simple polish motion along XY plane in world frame, otherwise hold robot motion in non-force-control axes",
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
        ">>> Tutorial description <<<\nThis tutorial runs non-real-time Cartesian-space unified "
        "motion-force control. The Z axis of the chosen reference frame will be activated for "
        "explicit force control, while the rest axes in the same reference frame will stay motion "
        "controlled.\n"
    )

    # The reference frame to use, see Robot::SendCartesianMotionForce() for more details
    force_ctrl_frame = flexivrdk.CoordType.WORLD
    if args.TCP:
        logger.info("Reference frame used for force control: robot TCP frame")
        force_ctrl_frame = flexivrdk.CoordType.TCP
    else:
        logger.info("Reference frame used for force control: robot world frame")

    # Whether to enable polish motion
    if args.polish:
        logger.info(
            "Robot will run a polish motion along XY plane in robot world frame"
        )
    else:
        logger.info("Robot will hold its motion in all non-force-controlled axes")

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

        # Wait for primitive to finish
        while not robot.primitive_states()["terminated"]:
            time.sleep(1)
        logger.info("Sensor zeroing complete")

        # Search for Contact
        # =========================================================================================
        # NOTE: there are several ways to do contact search, such as using primitives, or real-time
        # and non-real-time direct motion controls, etc. Here we use non-real-time direct Cartesian
        # control for example.
        logger.info("Searching for contact ...")

        # Set initial pose to current TCP pose
        init_pose = robot.states().tcp_pose.copy()
        logger.info(
            f"Initial TCP pose set to {init_pose} (position 3x1, rotation (quaternion) 4x1)"
        )

        # Use non-real-time mode to make the robot go to a set point with its own motion generator
        robot.SwitchMode(mode.NRT_CARTESIAN_MOTION_FORCE)

        # Search for contact with max contact wrench set to a small value for making soft contact
        robot.SetMaxContactWrench(MAX_WRENCH_FOR_CONTACT_SEARCH)

        # Set target point along -Z direction and expect contact to happen during the travel
        target_pose = init_pose.copy()
        target_pose[2] -= SEARCH_DISTANCE

        # Send target point to robot to start searching for contact and limit the velocity. Keep
        # target wrench 0 at this stage since we are not doing force control yet
        robot.SendCartesianMotionForce(target_pose, [0] * 6, [0] * 6, SEARCH_VELOCITY)

        # Use a while loop to poll robot states and check if a contact is made
        is_contacted = False
        while not is_contacted:
            # Compute norm of sensed external force applied on robot TCP
            ext_force = np.array(
                [
                    robot.states().ext_wrench_in_world[0],
                    robot.states().ext_wrench_in_world[1],
                    robot.states().ext_wrench_in_world[2],
                ]
            )

            # Contact is considered to be made if sensed TCP force exceeds the threshold
            if np.linalg.norm(ext_force) > PRESSING_FORCE:
                is_contacted = True
                logger.info("Contact detected at robot TCP")

            # Check at 1ms interval
            time.sleep(0.001)

        # Configure Force Control
        # =========================================================================================
        # Switch to non-real-time mode for discrete motion force control
        robot.SwitchMode(mode.NRT_CARTESIAN_MOTION_FORCE)

        # Set force control reference frame based on program argument. See function doc for more
        # details
        robot.SetForceControlFrame(force_ctrl_frame)

        # Set which Cartesian axis(s) to activate for force control. See function doc for more
        # details. Here we only active Z axis
        robot.SetForceControlAxis([False, False, True, False, False, False])

        # Uncomment the following line to enable passive force control, otherwise active force
        # control is used by default. See function doc for more details
        # robot.setPassiveForceControl(True)

        # NOTE: motion control always uses robot world frame, while force control can use
        # either world or TCP frame as reference frame

        # Start Unified Motion Force Control
        # =========================================================================================
        # Disable max contact wrench regulation. Need to do this AFTER the force control in Z axis
        # is activated (i.e. motion control disabled in Z axis) and the motion force control mode
        # is entered, this way the contact force along Z axis is explicitly regulated and will not
        # spike after the max contact wrench regulation for motion control is disabled
        robot.SetMaxContactWrench([float("inf")] * 6)

        # Update initial pose to current TCP pose
        init_pose = robot.states().tcp_pose.copy()
        logger.info(
            f"Initial TCP pose set to {init_pose} (position 3x1, rotation (quaternion) 4x1)"
        )

        # Periodic Task
        # =========================================================================================
        # Set loop period
        period = 1.0 / frequency
        logger.info(
            f"Sending command to robot at {frequency} Hz, or {period} seconds interval"
        )

        # Periodic loop counter
        loop_counter = 0

        # Send command periodically at user-specified frequency
        while True:
            # Use sleep to control loop period
            time.sleep(period)

            # Monitor fault on the connected robot
            if robot.fault():
                raise Exception("Fault occurred on the connected robot, exiting ...")

            # Initialize target pose to initial pose
            target_pose = init_pose.copy()

            # Set Fz according to reference frame to achieve a "pressing down" behavior
            Fz = 0.0
            if force_ctrl_frame == flexivrdk.CoordType.WORLD:
                Fz = PRESSING_FORCE
            elif force_ctrl_frame == flexivrdk.CoordType.TCP:
                Fz = -PRESSING_FORCE
            target_wrench = [0.0, 0.0, Fz, 0.0, 0.0, 0.0]

            # Apply constant force along Z axis of chosen reference frame, and do a simple polish
            # motion along XY plane in robot world frame
            if args.polish:
                # Create motion command to sine-sweep along Y direction
                target_pose[1] = init_pose[1] + SWING_AMP * math.sin(
                    2 * math.pi * SWING_FREQ * loop_counter * period
                )
                # Command target pose and target wrench
                robot.SendCartesianMotionForce(target_pose, target_wrench)
            # Apply constant force along Z axis of chosen reference frame, and hold motions in all
            # other axes
            else:
                # Command initial pose and target wrench
                robot.SendCartesianMotionForce(init_pose, target_wrench)

            # Increment loop counter
            loop_counter += 1

    except Exception as e:
        # Print exception error message
        logger.error(str(e))


if __name__ == "__main__":
    main()
