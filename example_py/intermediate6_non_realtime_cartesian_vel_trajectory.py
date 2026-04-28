#!/usr/bin/env python

"""intermediate6_non_realtime_cartesian_vel_trajectory.py

This tutorial demonstrates the recommended approach for continuous trajectory following in
non-real-time Cartesian-space motion control. The key concept is to integrate a desired
velocity into an *absolute* target pose rather than offsetting from the robot's measured
current pose, so the robot's internal trajectory generator always has a meaningful target
that grows further ahead over time.

Background – why "relative" (current_pose + delta) targets fail:
  In NRT_CARTESIAN_MOTION_FORCE mode the robot's built-in trajectory generator moves the arm
  from its *current* pose to the *commanded* target pose.  If you set the target each cycle
  as (robot.states().tcp_pose + small_delta), the generator is given a displacement that is
  only dt * v_desired ahead – typically well under 3 mm at 100 Hz – and may produce little or
  no useful motion, reversed motion, or large directional errors because the "current pose"
  sampled by the robot (at 1 kHz internally) can already be past the commanded target by the
  time the NRT command arrives.

Correct pattern (demonstrated here):
  1. Record the initial TCP pose as the *absolute* trajectory origin at t = 0.
  2. At every control cycle, advance the absolute target by (v_desired * dt) independently of
     the measured robot pose.
  3. Send this ever-advancing absolute target to SendCartesianMotionForce().
  The trajectory generator now always sees a target that is further ahead and produces smooth,
  predictable motion.

This pattern is suitable for:
  - Trajectory replay from recorded end-effector poses
  - Teleoperation (mirror an external arm's absolute joint/Cartesian state)
  - Policy deployment (execute predicted action chunks expressed as absolute poses)
  - Stylus / gamepad velocity control (integrate joystick velocity into absolute position)
"""

__copyright__ = "Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import math
import argparse
import spdlog  # pip install spdlog
import flexivrdk  # pip install flexivrdk


# Global constants
# ==================================================================================================
# Sine-sweep amplitude along X axis [m]
SWING_AMP = 0.1

# Sine-sweep frequency [Hz]
SWING_FREQ = 0.3

# Maximum Cartesian linear velocity for the trajectory generator [m/s]
MAX_LINEAR_VEL = 0.5

# Maximum Cartesian angular velocity for the trajectory generator [rad/s]
MAX_ANGULAR_VEL = 1.0


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
    args = argparser.parse_args()

    # Check if arguments are valid
    frequency = args.frequency
    assert frequency >= 1 and frequency <= 100, "Invalid <frequency> input"

    # Define alias
    logger = spdlog.ConsoleLogger("Example")
    mode = flexivrdk.Mode

    # Print description
    logger.info(
        ">>> Tutorial description <<<\n"
        "This tutorial demonstrates the correct velocity-integration pattern for continuous\n"
        "trajectory following using non-real-time Cartesian-space motion control.\n"
        "The robot TCP will sine-sweep along the X axis using an absolute target pose that\n"
        "is advanced each cycle from the initial pose – NOT from the measured current pose.\n"
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

        # Configure Motion Control
        # =========================================================================================
        # Switch to non-real-time Cartesian motion-force mode (pure motion control here)
        robot.SwitchMode(mode.NRT_CARTESIAN_MOTION_FORCE)

        # Disable force control on all axes so the robot does pure Cartesian motion control
        robot.SetForceControlAxis([False, False, False, False, False, False])

        # Capture the initial TCP pose ONCE and use it as the absolute trajectory origin.
        # IMPORTANT: do NOT update this reference from robot.states().tcp_pose inside the loop.
        # The absolute_target below is the only variable that tracks the desired position.
        init_pose = robot.states().tcp_pose.copy()
        logger.info(
            f"Initial TCP pose set to (position 3x1, quaternion 4x1): {init_pose}"
        )

        # Absolute target pose – starts at initial pose and is advanced every cycle
        absolute_target = init_pose.copy()

        # Periodic Task
        # =========================================================================================
        period = 1.0 / frequency
        loop_counter = 0
        logger.info(
            f"Sending commands to robot at {frequency} Hz ({period:.4f} s interval)\n"
            "Performing sine-sweep along X axis using absolute target integration ..."
        )

        while True:
            # Use sleep to control loop period
            time.sleep(period)

            # Monitor fault on the connected robot
            if robot.fault():
                raise Exception("Fault occurred on the connected robot, exiting ...")

            # --- Advance the ABSOLUTE target each cycle ---
            # Compute the desired X position as a function of elapsed time from the sine-sweep
            # formula.  The result is an absolute coordinate – it does NOT depend on the
            # robot's measured current pose.  This ensures the trajectory generator always
            # sees a target that is a meaningful distance ahead, producing smooth motion even
            # for very small step sizes (< 1 mm per cycle).
            elapsed = loop_counter * period
            absolute_target[0] = init_pose[0] + SWING_AMP * math.sin(
                2 * math.pi * SWING_FREQ * elapsed
            )
            # Orientation and other position axes remain at the initial values

            # Send the absolute target to the robot.
            # Velocity and wrench are left at zero here; the trajectory generator will plan the
            # motion profile automatically within the specified velocity/acceleration limits.
            # NOTE: if you want velocity feedforward to improve tracking at higher speeds, pass
            # the analytical derivative of your trajectory as the velocity argument:
            #   vel_x = SWING_AMP * 2 * pi * SWING_FREQ * cos(2 * pi * SWING_FREQ * elapsed)
            #   robot.SendCartesianMotionForce(absolute_target, [], [vel_x, 0, 0, 0, 0, 0],
            #                                 MAX_LINEAR_VEL, MAX_ANGULAR_VEL)
            robot.SendCartesianMotionForce(
                absolute_target, [], [], MAX_LINEAR_VEL, MAX_ANGULAR_VEL
            )

            # Increment loop counter
            loop_counter += 1

    except Exception as e:
        # Print exception error message
        logger.error(str(e))


if __name__ == "__main__":
    main()
