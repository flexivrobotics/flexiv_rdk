#!/usr/bin/env python

"""intermediate3_non_realtime_cartesian_motion_force_control.py

This tutorial runs real-time Cartesian-space unified motion-force control. The Z axis of the
chosen reference frame will be activated for explicit force control, while the rest axes in the
same reference frame will stay motion controlled.
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

# Pressing force to apply during the unified motion-force control [N]
PRESSING_FORCE = 5.0

# Cartesian linear velocity used to search for contact [m/s]
SEARCH_VELOCITY = 0.02

# Maximum distance to travel when searching for contact [m]
SEARCH_DISTANCE = 1.0

# Maximum contact wrench during contact search for soft contact
MAX_WRENCH_FOR_CONTACT_SEARCH = [10.0, 10.0, 10.0, 3.0, 3.0, 3.0]


def print_description():
    """
    Print tutorial description.

    """
    print(
        "This tutorial runs real-time Cartesian-space unified motion-force control. The Z "
        "axis of the chosen reference frame will be activated for explicit force control, "
        "while the rest axes in the same reference frame will stay motion controlled."
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
        "--TCP",
        action="store_true",
        help="use TCP frame as reference frame for force control, otherwise use base frame",
    )
    argparser.add_argument(
        "--polish",
        action="store_true",
        help="run a simple polish motion along XY plane in base frame, otherwise hold robot motion in non-force-control axes",
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

    # The reference frame to use, see Robot::sendCartesianMotionForce() for more details
    frame_str = "BASE"
    if args.TCP:
        log.info("Reference frame used for force control: robot TCP frame")
        frame_str = "TCP"
    else:
        log.info("Reference frame used for force control: robot base frame")

    # Whether to enable polish motion
    if args.polish:
        log.info("Robot will run a polish motion along XY plane in robot base frame")
    else:
        log.info("Robot will hold its motion in all non-force-controlled axes")

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

        # Search for Contact
        # =========================================================================================
        # NOTE: there are several ways to do contact search, such as using primitives, or real-time
        # and non-real-time direct motion controls, etc. Here we use non-real-time direct Cartesian
        # control for example.
        log.info("Searching for contact ...")

        # Set initial pose to current TCP pose
        robot.getRobotStates(robot_states)
        init_pose = robot_states.tcpPose.copy()
        print(
            "Initial TCP pose set to [position 3x1, rotation (quaternion) 4x1]: ",
            init_pose,
        )

        # Use non-real-time mode to make the robot go to a set point with its own motion generator
        robot.setMode(mode.NRT_CARTESIAN_MOTION_FORCE)

        # Search for contact with max contact wrench set to a small value for making soft contact
        robot.setMaxContactWrench(MAX_WRENCH_FOR_CONTACT_SEARCH)

        # Set target point along -Z direction and expect contact to happen during the travel
        target_pose = init_pose.copy()
        target_pose[2] -= SEARCH_DISTANCE

        # Send target point to robot to start searching for contact and limit the velocity. Keep
        # target wrench 0 at this stage since we are not doing force control yet
        robot.sendCartesianMotionForce(target_pose, [0] * 6, SEARCH_VELOCITY)

        # Use a while loop to poll robot states and check if a contact is made
        is_contacted = False
        while not is_contacted:
            # Get the latest robot states
            robot.getRobotStates(robot_states)

            # Compute norm of sensed external force applied on robot TCP
            ext_force = np.array(
                [
                    robot_states.extWrenchInBase[0],
                    robot_states.extWrenchInBase[1],
                    robot_states.extWrenchInBase[2],
                ]
            )

            # Contact is considered to be made if sensed TCP force exceeds the threshold
            if np.linalg.norm(ext_force) > PRESSING_FORCE:
                is_contacted = True
                log.info("Contact detected at robot TCP")

            # Check at 1ms interval
            time.sleep(0.001)

        # Configure Force Control
        # =========================================================================================
        # The force control configurations can only be updated when the robot is in IDLE mode
        robot.stop()

        # Set force control reference frame based on program argument. See function doc for more
        # details
        robot.setForceControlFrame(frame_str)

        # Set which Cartesian axis(s) to activate for force control. See function doc for more
        # details. Here we only active Z axis
        robot.setForceControlAxis([False, False, True, False, False, False])

        # Uncomment the following line to enable passive force control, otherwise active force
        # control is used by default. See function doc for more details
        # robot.setPassiveForceControl(True)

        # NOTE: motion control always uses robot base frame, while force control can use
        # either base or TCP frame as reference frame

        # Start Unified Motion Force Control
        # =========================================================================================
        # Switch to non-real-time mode for discrete motion force control
        robot.setMode(mode.NRT_CARTESIAN_MOTION_FORCE)

        # Disable max contact wrench regulation. Need to do this AFTER the force control in Z axis
        # is activated (i.e. motion control disabled in Z axis) and the motion force control mode
        # is entered, this way the contact force along Z axis is explicitly regulated and will not
        # spike after the max contact wrench regulation for motion control is disabled
        robot.resetMaxContactWrench()

        # Update initial pose to current TCP pose
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
        print(
            "Sending command to robot at",
            frequency,
            "Hz, or",
            period,
            "seconds interval",
        )

        # Periodic loop counter
        loop_counter = 0

        # Send command periodically at user-specified frequency
        while True:
            # Use sleep to control loop period
            time.sleep(period)

            # Monitor fault on robot server
            if robot.isFault():
                raise Exception("Fault occurred on robot server, exiting ...")

            # Initialize target pose to initial pose
            target_pose = init_pose.copy()

            # Set Fz according to reference frame to achieve a "pressing down" behavior
            Fz = 0.0
            if frame_str == "BASE":
                Fz = -PRESSING_FORCE
            elif frame_str == "TCP":
                Fz = PRESSING_FORCE
            target_wrench = [0.0, 0.0, Fz, 0.0, 0.0, 0.0]

            # Apply constant force along Z axis of chosen reference frame, and do a simple polish
            # motion along XY plane in robot base frame
            if args.polish:
                # Create motion command to sine-sweep along Y direction
                target_pose[1] = init_pose[1] + SWING_AMP * math.sin(
                    2 * math.pi * SWING_FREQ * loop_counter * period
                )
                # Command target pose and target wrench
                robot.sendCartesianMotionForce(target_pose, target_wrench)
            # Apply constant force along Z axis of chosen reference frame, and hold motions in all
            # other axes
            else:
                # Command initial pose and target wrench
                robot.sendCartesianMotionForce(init_pose, target_wrench)

            # Increment loop counter
            loop_counter += 1

    except Exception as e:
        # Print exception error message
        log.error(str(e))


if __name__ == "__main__":
    main()
